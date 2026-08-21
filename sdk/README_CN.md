# SDK 说明文档 #
蓝海2D激光雷达的Windows/Linux SDK及示例程序

# 编译与使用指南 #

## LINUX 系统 ##
前置条件：必须安装g++和gcc编译器

	cmake CMakeLists.txt
	make
	make install
	./demo config/xxxx.txt  

## WINDOWS 系统 ##
使用Windows PowerShell

    使用cmake_gui生成工程，打开项目并编译
	详细操作请参考 docs/SDK开发手册.docx
使用Visual Studio 2013或更高版本打开工程，生成解决方案 

**特别说明：请参考对应产品的说明文档选择指定的配置文件，并确保硬件正常工作后，软件才能正常调用。**

## 配置文件从Windows转换到Linux ##
	vim xxxx.txt
	set ff=unix

