#!/bin/bash

# 检查是否有root权限
if [ "$(id -u)" != "0" ]; then
    echo "请以root用户身份运行此脚本。"
    exit 1
fi

# 判断CPU架构并设置IOMMU选项
iommu_option=""
if grep -qi "vendor_id.*GenuineIntel" /proc/cpuinfo; then
    echo "检测到Intel CPU"
    iommu_option="intel_iommu=on"
elif grep -qi "vendor_id.*AuthenticAMD" /proc/cpuinfo; then
    echo "检测到AMD CPU"
    iommu_option="amd_iommu=on"
else
    echo "无法识别CPU架构，退出。"
    exit 1
fi

#配置文件后，需要sync同步，确保将内容写入到了配置文件中
sync

# 备份当前GRUB配置文件
echo "备份当前GRUB配置文件..."
sudo cp /etc/default/grub /etc/default/grub.backup

# 安全地向GRUB配置添加IOMMU选项
if [ -n "$iommu_option" ]; then
    # 使用正则表达式检查IOMMU选项是否已存在，避免重复添加
    if ! grep -qP "(?<=^GRUB_CMDLINE_LINUX=).*\b${iommu_option}\b(?=\")" /etc/default/grub; then
        # 添加IOMMU选项到GRUB_CMDLINE_LINUX
        sudo sed -i "s/^GRUB_CMDLINE_LINUX=\"\(.*\)\"$/GRUB_CMDLINE_LINUX=\"\1 ${iommu_option}\"/" /etc/default/grub
        echo "已向GRUB_CMDLINE_LINUX添加'$iommu_option'"
    else
        echo "IOMMU选项 '$iommu_option' 已经存在于GRUB配置中，无需重复添加。"
        exit 1
    fi
    
    echo "更新GRUB配置。"
    sudo update-grub
	
	# 提醒用户重启系统以应用更改
    echo "已成功修改GRUB配置（如需修改）。请重启系统以应用更改。"
    read -p "是否现在重启系统？(y/n) " REBOOT_NOW
    if [[ $REBOOT_NOW == [yY] ]]; then
        echo "正在重启系统..."
        sudo reboot
    else
        echo "未选择重启，记得手动重启以应用更改。"
    fi
else
    echo "未识别到有效的IOMMU选项。"
fi
