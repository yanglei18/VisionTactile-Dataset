; All values are passed by build.py, never filled in manually for a release.
#ifndef AppVersion
  #error AppVersion is required; use build.ps1
#endif
#ifndef SourceDir
  #error SourceDir is required; use build.ps1
#endif
#ifndef ReleaseDir
  #error ReleaseDir is required; use build.ps1
#endif

[Setup]
AppId=VisionTactile.Tactile3D.Windows.x64
AppName={cm:ProductName}
AppVersion={#AppVersion}
AppPublisher=Vision Tactile
VersionInfoDescription=Tactile 3D Viewer Setup
VersionInfoProductName=Tactile 3D Viewer
DefaultDirName={localappdata}\Programs\VisionTactile\Tactile3D
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
UninstallDisplayIcon={app}\VisionTactile3D.exe
OutputDir={#ReleaseDir}
OutputBaseFilename=VisionTactile3D-{#AppVersion}-windows-x64-setup
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
ShowLanguageDialog=yes
LanguageDetectionMethod=uilanguage
UsePreviousLanguage=yes
SetupLogging=yes

[Languages]
Name: "en"; MessagesFile: "compiler:Default.isl"
Name: "zh_CN"; MessagesFile: "vendor\ChineseSimplified.isl"

[CustomMessages]
en.ProductName=Tactile 3D Viewer
zh_CN.ProductName=触觉三维预览
en.DesktopShortcut=Create a desktop shortcut
zh_CN.DesktopShortcut=创建桌面快捷方式
en.UninstallShortcut=Uninstall Tactile 3D Viewer
zh_CN.UninstallShortcut=卸载触觉三维预览
en.LaunchApplication=Launch Tactile 3D Viewer
zh_CN.LaunchApplication=启动触觉三维预览

[Tasks]
Name: "desktopicon"; Description: "{cm:DesktopShortcut}"

[Files]
Source: "{#SourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[InstallDelete]
; Replace this product's known shortcuts when the installer language changes.
Type: files; Name: "{autodesktop}\触觉三维预览.lnk"
Type: files; Name: "{autodesktop}\Tactile 3D Viewer.lnk"
Type: files; Name: "{group}\触觉三维预览.lnk"
Type: files; Name: "{group}\Tactile 3D Viewer.lnk"
Type: files; Name: "{group}\卸载触觉三维预览.lnk"
Type: files; Name: "{group}\Uninstall Tactile 3D Viewer.lnk"

[Icons]
Name: "{group}\{cm:ProductName}"; Filename: "{app}\VisionTactile3D.exe"; WorkingDir: "{app}"
Name: "{autodesktop}\{cm:ProductName}"; Filename: "{app}\VisionTactile3D.exe"; WorkingDir: "{app}"; Tasks: desktopicon
Name: "{group}\{cm:UninstallShortcut}"; Filename: "{uninstallexe}"

[Run]
Filename: "{app}\VisionTactile3D.exe"; Description: "{cm:LaunchApplication}"; Flags: nowait postinstall skipifsilent

; User settings live outside {app} and remain available after upgrades/uninstall.
; Inno records installed files itself; never recursively delete user directories.
