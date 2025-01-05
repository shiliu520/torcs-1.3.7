# Execute the following commands when compiling the first compilation
# git clean -fdx
# git checkout .

./setup_win32_debug.bat
./setup_win32-data-from-CVS_debug.bat

$vsIdeInstallPath = "D:\Softwares\VisualStudio\2012\Common7\IDE"
$devenvPath = Join-Path $vsIdeInstallPath "devenv.exe"
$torcsInstallPath = $PSScriptRoot
$solutionPath = Join-Path $torcsInstallPath "TORCS.sln"
$projectFilePath = Join-Path $torcsInstallPath "src\libs\client\client.vcxproj"

# 检查是否需要升级solution
$vs2012ExpectedVersion = "12.00"
$versionRegex = "Format Version (\d+\.\d+)"
$matchResult = Select-String -Path $solutionPath -Pattern $versionRegex

$process_ = [PSCustomObject]@{
    ExitCode = $null
    ProcessName = ""
    StartTime = $null
}
if ($matchResult) {
    $foundVersion = $matchResult.Matches.Groups[1].Value
    Write-Host $foundVersion

    # 这里假设Visual Studio 2022期望的格式版本是12.00
    $expectedVersion = $vs2012ExpectedVersion
    if ([double]$foundVersion -lt [double]$expectedVersion) {
        Write-Host "该解决方案需要升级, 升级中, 请等待......"
        $process = Start-Process -FilePath $devenvPath -ArgumentList $solutionPath, "/upgrade" -Wait -PassThru
    } else {
        Write-Host "该解决方案不需要升级."

        $process = $process_
        $process.ExitCode = -1
    }
} else {
    Write-Host "无法确定解决方案的格式版本."
    $process = $process_
    $process.ExitCode = -2
}

# 检查进程的退出码，以确认是否成功执行
if ($process.ExitCode -eq 0) {
    Write-Host "devenv.exe升级解决方案成功, 继续执行增加编译选项指令..."

    $xml = [xml](Get-Content $projectFilePath)

    # 查找 Link 节点，在 Link 节点下操作
    $LinkNodes = $xml.Project.ItemDefinitionGroup | Where-Object { $_.Link }

    foreach ($LinkNode in $LinkNodes) {
        # 检查 Link 节点下是否已经存在 AdditionalOptions 元素
        $existingOption = $LinkNode.Link.AdditionalOptions
        if ($existingOption) {
            $newOption = $existingOption.InnerText + " /SAFESEH:NO"
            $existingOption.InnerText = $newOption
        } else {
            $newOptionElement = $xml.CreateElement("AdditionalOptions")
            $newOptionElement.InnerText = "/SAFESEH:NO"
            $LinkNode.Link.AppendChild($newOptionElement)
        }
    }

    # 保存修改后的 xml 文件
    $xml = [xml] $xml.OuterXml.Replace(" xmlns=`"`"", "")
    $xml.Save($projectFilePath)
} elseif ($process.ExitCode -eq -1) {
    Write-Host "也不需要增加编译选项指令."
} elseif ($process.ExitCode -eq -2) {
    Write-Host "不需要增加编译选项指令."
}
else {
    Write-Host "devenv.exe升级解决方案出现问题, 退出码为 $($process.ExitCode)."
}

# Write-Host "编译项目中..."
# $arguments = "/Build", "Debug", $solutionPath

# try {
#     $process = Start-Process -FilePath $devenvPath -ArgumentList $arguments -Wait -PassThru -NoNewWindow
#     if ($process.ExitCode -eq 0) {
#         Write-Host "解决方案编译成功。"
#     } else {
#         Write-Host "解决方案编译失败，退出码: $($process.ExitCode)"
#     }
# } catch {
#     Write-Host "编译过程出现错误: $($_.Exception.Message)"
# }