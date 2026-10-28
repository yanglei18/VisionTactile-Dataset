; Product identity is deliberately separate from the existing Tactile 3D Viewer.
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
AppId=VisionTactile.HandMotionTouch.Windows.x64
AppName={cm:ProductName}
AppVersion={#AppVersion}
AppPublisher=Vision Tactile
VersionInfoDescription=Hand Motion and Touch Viewer Setup
VersionInfoProductName=Hand Motion and Touch Viewer
DefaultDirName={localappdata}\Programs\VisionTactile\HandMotionTouch
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
UninstallDisplayIcon={app}\VisionHandMotionTouch.exe
OutputDir={#ReleaseDir}
OutputBaseFilename=VisionHandMotionTouch-{#AppVersion}-windows-x64-setup
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
en.ProductName=Hand Motion and Touch Viewer
zh_CN.ProductName=手势与触觉可视化
en.DesktopShortcut=Create a desktop shortcut
zh_CN.DesktopShortcut=创建桌面快捷方式
en.UninstallShortcut=Uninstall Hand Motion and Touch Viewer
zh_CN.UninstallShortcut=卸载手势与触觉可视化
en.LaunchApplication=Launch Hand Motion and Touch Viewer
zh_CN.LaunchApplication=启动手势与触觉可视化

[Tasks]
Name: "desktopicon"; Description: "{cm:DesktopShortcut}"

[Files]
Source: "{#SourceDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs

[InstallDelete]
; Update only this product's shortcuts when the installation language changes.
Type: files; Name: "{autodesktop}\手势与触觉可视化.lnk"
Type: files; Name: "{autodesktop}\Hand Motion and Touch Viewer.lnk"
Type: files; Name: "{group}\手势与触觉可视化.lnk"
Type: files; Name: "{group}\Hand Motion and Touch Viewer.lnk"
Type: files; Name: "{group}\卸载手势与触觉可视化.lnk"
Type: files; Name: "{group}\Uninstall Hand Motion and Touch Viewer.lnk"

[Icons]
Name: "{group}\{cm:ProductName}"; Filename: "{app}\VisionHandMotionTouch.exe"; WorkingDir: "{app}"
Name: "{autodesktop}\{cm:ProductName}"; Filename: "{app}\VisionHandMotionTouch.exe"; WorkingDir: "{app}"; Tasks: desktopicon
Name: "{group}\{cm:UninstallShortcut}"; Filename: "{uninstallexe}"

[Run]
Filename: "{app}\VisionHandMotionTouch.exe"; Description: "{cm:LaunchApplication}"; Flags: nowait postinstall skipifsilent

; Settings and calibrations are outside {app}; never recursively delete them.
