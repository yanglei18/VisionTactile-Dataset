; Each hand owns a distinct installation, shortcuts, and upgrade identity.
#ifndef AppVersion
  #error AppVersion is required; use build.ps1
#endif
#ifndef SourceDir
  #error SourceDir is required; use build.ps1
#endif
#ifndef ReleaseDir
  #error ReleaseDir is required; use build.ps1
#endif

#ifndef HandSide
  #define HandSide "Right"
#endif
#if HandSide == "Left"
  #define HandZh "左"
  #define OtherHand "Right"
#elif HandSide == "Right"
  #define HandZh "右"
  #define OtherHand "Left"
#else
  #error HandSide must be Left or Right
#endif

[Setup]
AppId=VisionTactile.{#HandSide}HandDisplay.Windows.x64
AppName={cm:ProductName}
AppVersion={#AppVersion}
AppPublisher=Vision Tactile
VersionInfoDescription={#HandSide} Hand 3D Display Setup
VersionInfoProductName={#HandSide} Hand 3D Display
DefaultDirName={localappdata}\Programs\VisionTactile\{#HandSide}HandDisplay
DefaultGroupName={cm:ProductName}
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible and not arm64
ArchitecturesInstallIn64BitMode=x64compatible
MinVersion=10.0.17763
DisableProgramGroupPage=yes
DisableWelcomePage=yes
DisableReadyPage=yes
DisableDirPage=auto
UsePreviousAppDir=yes
UsePreviousTasks=yes
CloseApplications=yes
RestartApplications=no
UninstallDisplayIcon={app}\Vision{#HandSide}HandDisplay.exe
OutputDir={#ReleaseDir}
OutputBaseFilename=Vision{#HandSide}HandDisplay-{#AppVersion}-windows-x64-setup
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
ShowLanguageDialog=yes
LanguageDetectionMethod=uilanguage
UsePreviousLanguage=yes
SetupLogging=yes

[Languages]
Name: "en"; MessagesFile: "compiler:Default.isl"
Name: "zh_CN"; MessagesFile: "..\..\tactile_viewer_windows\vendor\ChineseSimplified.isl"

[CustomMessages]
en.ProductName={#HandSide} Hand 3D Display
zh_CN.ProductName={#HandZh}手三维展示
en.DesktopShortcut=Create a desktop shortcut
zh_CN.DesktopShortcut=创建桌面快捷方式
en.UninstallShortcut=Uninstall {#HandSide} Hand 3D Display
zh_CN.UninstallShortcut=卸载{#HandZh}手三维展示
en.LaunchApplication=Launch {#HandSide} Hand 3D Display
zh_CN.LaunchApplication=启动{#HandZh}手三维展示

en.OtherHandDirectory=The other hand display is installed here. Choose a separate installation folder.
zh_CN.OtherHandDirectory=另一只手的展示程序已安装在此处，请选择独立的安装目录。

[Tasks]
Name: "desktopicon"; Description: "{cm:DesktopShortcut}"

[Files]
Source: "{#SourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[InstallDelete]
; Update only this product's shortcuts when the installation language changes.
Type: files; Name: "{autodesktop}\{#HandZh}手三维展示.lnk"
Type: files; Name: "{autodesktop}\{#HandSide} Hand 3D Display.lnk"
Type: files; Name: "{group}\{#HandZh}手三维展示.lnk"
Type: files; Name: "{group}\{#HandSide} Hand 3D Display.lnk"
Type: files; Name: "{group}\卸载{#HandZh}手三维展示.lnk"
Type: files; Name: "{group}\Uninstall {#HandSide} Hand 3D Display.lnk"

[Icons]
Name: "{group}\{cm:ProductName}"; Filename: "{app}\Vision{#HandSide}HandDisplay.exe"; WorkingDir: "{app}"
Name: "{autodesktop}\{cm:ProductName}"; Filename: "{app}\Vision{#HandSide}HandDisplay.exe"; WorkingDir: "{app}"; Tasks: desktopicon
Name: "{group}\{cm:UninstallShortcut}"; Filename: "{uninstallexe}"

[Run]
Filename: "{app}\Vision{#HandSide}HandDisplay.exe"; Description: "{cm:LaunchApplication}"; Flags: nowait postinstall skipifsilent

; Settings and calibrations are outside {app}; never recursively delete them.


[Code]
function PrepareToInstall(var NeedsRestart: Boolean): String;
begin
  Result := '';
  if FileExists(AddBackslash(WizardDirValue) + 'Vision{#OtherHand}HandDisplay.exe') then
    Result := CustomMessage('OtherHandDirectory');
end;
