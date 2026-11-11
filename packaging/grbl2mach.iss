; Inno Setup script of the GRBL2MACH Windows installer. Built by build_installer.ps1.
;
; Installed for the current user, without administrator rights: the bridge writes its settings
; (EEPROM.DAT) and the machine profile (MACH3.xml) next to the program.

#ifndef AppVersion
  #define AppVersion "0.0.0"
#endif
#ifndef BuildDir
  #define BuildDir "..\build"
#endif
#ifndef OutDir
  #define OutDir "..\release"
#endif

[Setup]
AppId={{B7C3E0A2-5F14-4E8B-9A53-2D7F1C6E8A10}
AppName=GRBL2MACH
AppVersion={#AppVersion}
AppVerName=GRBL2MACH {#AppVersion}
AppPublisher=GRBL2MACH contributors
AppPublisherURL=https://github.com/manuprendlair/GRBL2MACH
AppSupportURL=https://github.com/manuprendlair/GRBL2MACH/issues
DefaultDirName={localappdata}\Programs\GRBL2MACH
DefaultGroupName=GRBL2MACH
DisableProgramGroupPage=yes
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
MinVersion=10.0
OutputDir={#OutDir}
OutputBaseFilename=GRBL2MACH-{#AppVersion}-setup
SetupIconFile=grbl2mach.ico
WizardSmallImageFile=grbl2mach-512.png
UninstallDisplayIcon={app}\grbl2mach.exe
LicenseFile=..\COPYING
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
CloseApplications=yes
VersionInfoVersion={#AppVersion}

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
Name: "french"; MessagesFile: "compiler:Languages\French.isl"

[CustomMessages]
english.ProfileTitle=Mach3 machine profile
french.ProfileTitle=Profil machine Mach3
english.ProfileDesc=GRBL2MACH reads your machine's settings from your Mach3 profile.
french.ProfileDesc=GRBL2MACH lit les réglages de votre machine dans votre profil Mach3.
english.ProfileSub=Choose your Mach3 profile (for example C:\Mach3\Mach3Mill.xml). A copy is made next to the program: your Mach3 profile is not modified.
french.ProfileSub=Choisissez votre profil Mach3 (par exemple C:\Mach3\Mach3Mill.xml). Une copie est faite à côté du programme : votre profil Mach3 n'est pas modifié.
english.ProfileLabel=Mach3 profile (.xml):
french.ProfileLabel=Profil Mach3 (.xml) :
english.ProfileNotFound=This file does not exist.
french.ProfileNotFound=Ce fichier n'existe pas.
english.ProfileEmpty=Without a profile, GRBL2MACH cannot start. You will have to copy your profile as MACH3.xml into the installation folder. Continue?
french.ProfileEmpty=Sans profil, GRBL2MACH ne pourra pas démarrer. Il faudra copier votre profil sous le nom MACH3.xml dans le dossier d'installation. Continuer ?

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"

[Files]
Source: "{#BuildDir}\grbl2mach.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "grbl2mach-start.cmd"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\README.md"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\COPYING"; DestDir: "{app}"; Flags: ignoreversion
Source: "..\tools\mach3_editor.html"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\GRBL2MACH"; Filename: "{app}\grbl2mach.exe"; WorkingDir: "{app}"
Name: "{group}\GRBL2MACH (console)"; Filename: "{app}\grbl2mach-start.cmd"; WorkingDir: "{app}"; IconFilename: "{app}\grbl2mach.exe"
Name: "{group}\Mach3 profile editor"; Filename: "{app}\mach3_editor.html"
Name: "{group}\{cm:UninstallProgram,GRBL2MACH}"; Filename: "{uninstallexe}"
Name: "{userdesktop}\GRBL2MACH"; Filename: "{app}\grbl2mach.exe"; WorkingDir: "{app}"; Tasks: desktopicon

[Code]
var
  ProfilePage: TInputFileWizardPage;

function HasProfile: Boolean;
begin
  Result := FileExists(ExpandConstant('{app}\MACH3.xml'));
end;

procedure InitializeWizard;
begin
  ProfilePage := CreateInputFilePage(wpSelectDir, CustomMessage('ProfileTitle'),
    CustomMessage('ProfileDesc'), CustomMessage('ProfileSub'));
  ProfilePage.Add(CustomMessage('ProfileLabel'), 'XML (*.xml)|*.xml|*.*|*.*', '.xml');
  ProfilePage.Values[0] := ExpandConstant('{param:profile|}');
  if (ProfilePage.Values[0] = '') and FileExists('C:\Mach3\Mach3Mill.xml') then
    ProfilePage.Values[0] := 'C:\Mach3\Mach3Mill.xml';
end;

// An update keeps the profile already installed
function ShouldSkipPage(PageID: Integer): Boolean;
begin
  Result := (PageID = ProfilePage.ID) and HasProfile;
end;

function NextButtonClick(CurPageID: Integer): Boolean;
begin
  Result := True;
  if CurPageID = ProfilePage.ID then begin
    if ProfilePage.Values[0] = '' then
      Result := SuppressibleMsgBox(CustomMessage('ProfileEmpty'), mbConfirmation, MB_YESNO, IDYES) = IDYES
    else if not FileExists(ProfilePage.Values[0]) then begin
      SuppressibleMsgBox(CustomMessage('ProfileNotFound'), mbError, MB_OK, IDOK);
      Result := False;
    end;
  end;
end;

procedure CurStepChanged(CurStep: TSetupStep);
begin
  if (CurStep = ssPostInstall) and not HasProfile and (ProfilePage.Values[0] <> '') and FileExists(ProfilePage.Values[0]) then
    CopyFile(ProfilePage.Values[0], ExpandConstant('{app}\MACH3.xml'), True);
end;
