/*
  rnr_hid_win.c - Windows HID transport to the RNR Eco Motion 2.0 controller

  Finds the controller by VID/PID through SetupAPI and talks to it with
  overlapped ReadFile/WriteFile, like the vendor plugin (hidapi) does.
  Frames are written as is, starting at byte 0 (buf[0] = 0x02 / 0x03), as
  bench-tested by BreakoutCNC; buffers are sized to the report lengths
  reported by the HID driver.

  Part of GRBL2MACH. GPL-3.0-or-later.
*/

#ifdef _WIN32

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <windows.h>
#include <setupapi.h>
#include <hidsdi.h>

#include "rnr_transport.h"

#define WRITE_TIMEOUT_MS 100

typedef struct {
    HANDLE h;
    HANDLE read_event;
    HANDLE write_event;
    OVERLAPPED read_ol;
    bool read_pending;
    DWORD in_len, out_len;
    uint8_t rx[512];
    uint8_t tx[512];
} hid_ctx_t;

static bool is_disconnect (DWORD err)
{
    return err == ERROR_DEVICE_NOT_CONNECTED || err == ERROR_GEN_FAILURE ||
            err == ERROR_INVALID_HANDLE || err == ERROR_BAD_COMMAND || err == ERROR_OPERATION_ABORTED;
}

static int hid_write (rnr_transport_t *t, const uint8_t buf[RNR_OUT_BYTES])
{
    hid_ctx_t *c = t->ctx;
    OVERLAPPED ol = {0};
    DWORD written = 0;

    memset(c->tx, 0, c->out_len);
    memcpy(c->tx, buf, RNR_OUT_BYTES);
    ol.hEvent = c->write_event;
    ResetEvent(ol.hEvent);

    if(!WriteFile(c->h, c->tx, c->out_len, &written, &ol)) {
        DWORD err = GetLastError();
        if(err != ERROR_IO_PENDING)
            return is_disconnect(err) ? RNR_IO_DISCONNECTED : RNR_IO_FAILED;
        if(WaitForSingleObject(ol.hEvent, WRITE_TIMEOUT_MS) != WAIT_OBJECT_0) {
            CancelIoEx(c->h, &ol);
            GetOverlappedResult(c->h, &ol, &written, TRUE);  // the kernel must be done with ol
            return RNR_IO_FAILED;
        }
        if(!GetOverlappedResult(c->h, &ol, &written, FALSE))
            return is_disconnect(GetLastError()) ? RNR_IO_DISCONNECTED : RNR_IO_FAILED;
    }

    return written == c->out_len ? RNR_IO_OK : RNR_IO_FAILED;
}

static int hid_read (rnr_transport_t *t, uint8_t buf[RNR_IN_BYTES], uint32_t timeout_ms)
{
    hid_ctx_t *c = t->ctx;
    DWORD got = 0;

    if(!c->read_pending) {
        memset(&c->read_ol, 0, sizeof(c->read_ol));
        c->read_ol.hEvent = c->read_event;
        ResetEvent(c->read_event);
        if(ReadFile(c->h, c->rx, c->in_len, &got, &c->read_ol))
            goto done;
        if(GetLastError() != ERROR_IO_PENDING)
            return RNR_IO_DISCONNECTED;
        c->read_pending = true;
    }

    if(WaitForSingleObject(c->read_event, timeout_ms) != WAIT_OBJECT_0)
        return RNR_IO_TIMEOUT;     // the read stays pending and completes on a later call

    c->read_pending = false;
    if(!GetOverlappedResult(c->h, &c->read_ol, &got, FALSE))
        return is_disconnect(GetLastError()) ? RNR_IO_DISCONNECTED : RNR_IO_TIMEOUT;

done:
    if(got < RNR_IN_BYTES)
        return RNR_IO_TIMEOUT;
    memcpy(buf, c->rx, RNR_IN_BYTES);

    return RNR_IO_OK;
}

static void hid_close (rnr_transport_t *t)
{
    hid_ctx_t *c = t->ctx;
    DWORD ignored;

    CancelIoEx(c->h, NULL);
    if(c->read_pending)
        GetOverlappedResult(c->h, &c->read_ol, &ignored, TRUE);
    CloseHandle(c->h);
    CloseHandle(c->read_event);
    CloseHandle(c->write_event);
    free(c);
    free(t);
}

// Returns the device path of the first controller found, or false.
static bool find_device (char *path, size_t path_len, char *info, size_t info_len)
{
    GUID guid;
    HidD_GetHidGuid(&guid);
    HDEVINFO set = SetupDiGetClassDevsA(&guid, NULL, NULL, DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    SP_DEVICE_INTERFACE_DATA iface = { .cbSize = sizeof(SP_DEVICE_INTERFACE_DATA) };
    bool found = false;

    if(set == INVALID_HANDLE_VALUE)
        return false;

    for(DWORD i = 0; !found && SetupDiEnumDeviceInterfaces(set, NULL, &guid, i, &iface); i++) {
        DWORD size = 0;
        SetupDiGetDeviceInterfaceDetailA(set, &iface, NULL, 0, &size, NULL);
        SP_DEVICE_INTERFACE_DETAIL_DATA_A *detail = calloc(1, size ? size : 1);
        if(!detail)
            continue;
        detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);
        if(SetupDiGetDeviceInterfaceDetailA(set, &iface, detail, size, NULL, NULL)) {
            // Zero access: query attributes without claiming the device
            HANDLE h = CreateFileA(detail->DevicePath, 0, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, 0, NULL);
            if(h != INVALID_HANDLE_VALUE) {
                HIDD_ATTRIBUTES attr = { .Size = sizeof(HIDD_ATTRIBUTES) };
                if(HidD_GetAttributes(h, &attr) && attr.VendorID == RNR_VID && attr.ProductID == RNR_PID) {
                    wchar_t product[128] = {0}, serial[128] = {0};
                    HidD_GetProductString(h, product, sizeof(product));
                    HidD_GetSerialNumberString(h, serial, sizeof(serial));
                    snprintf(info, info_len, "%ls, serial %ls, VID 0x%04X PID 0x%04X rev 0x%04X",
                              product, serial, attr.VendorID, attr.ProductID, attr.VersionNumber);
                    snprintf(path, path_len, "%s", detail->DevicePath);
                    found = true;
                }
                CloseHandle(h);
            }
        }
        free(detail);
    }
    SetupDiDestroyDeviceInfoList(set);

    return found;
}

rnr_transport_t *rnr_hid_open (char *info, size_t info_len)
{
    char path[512];
    rnr_transport_t *t;
    hid_ctx_t *c;
    PHIDP_PREPARSED_DATA pp;
    HIDP_CAPS caps;

    info[0] = '\0';
    if(!find_device(path, sizeof(path), info, info_len))
        return NULL;

    t = calloc(1, sizeof(rnr_transport_t));
    c = calloc(1, sizeof(hid_ctx_t));
    if(!t || !c)
        goto fail;

    c->h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);
    if(c->h == INVALID_HANDLE_VALUE)
        goto fail;

    c->in_len = RNR_IN_BYTES;
    c->out_len = RNR_OUT_BYTES;
    if(HidD_GetPreparsedData(c->h, &pp)) {
        if(HidP_GetCaps(pp, &caps) == HIDP_STATUS_SUCCESS) {
            if(caps.InputReportByteLength >= RNR_IN_BYTES && caps.InputReportByteLength <= sizeof(c->rx))
                c->in_len = caps.InputReportByteLength;
            if(caps.OutputReportByteLength >= RNR_OUT_BYTES && caps.OutputReportByteLength <= sizeof(c->tx))
                c->out_len = caps.OutputReportByteLength;
        }
        HidD_FreePreparsedData(pp);
    }
    size_t n = strlen(info);
    snprintf(info + n, info_len - n, ", reports in %lu / out %lu bytes", (unsigned long)c->in_len, (unsigned long)c->out_len);

    HidD_SetNumInputBuffers(c->h, 64);
    c->read_event = CreateEventW(NULL, TRUE, FALSE, NULL);
    c->write_event = CreateEventW(NULL, TRUE, FALSE, NULL);

    t->name = "usb";
    t->write = hid_write;
    t->read = hid_read;
    t->close = hid_close;
    t->ctx = c;

    return t;

fail:
    if(c && c->h && c->h != INVALID_HANDLE_VALUE)
        CloseHandle(c->h);
    free(c);
    free(t);
    return NULL;
}

#else

#include "rnr_transport.h"

rnr_transport_t *rnr_hid_open (char *info, size_t info_len)
{
    (void)info_len;
    info[0] = '\0';
    return NULL; // TODO: hidapi transport for Linux / macOS
}

#endif
