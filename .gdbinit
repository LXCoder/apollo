###############################################
#  基本配置
###############################################
echo *** GDBINIT LOADED ***\n
# 启用彩色提示（gdb ≥ 8.0）
#set style on

# 遇到信号停止
set pagination off
set confirm off

# 调试输出时自动打印线程切换信息
set print thread-events on

# 打印时显示结构体字段名
set print pretty on
set print array on
set print elements 200

# 自动保存历史
set history save on
set history filename ~/.gdb_history
set history size 5000

###############################################
#  工程路径（请修改）
###############################################

# 项目源码根目录（你要调试的源码）
# 例如：/home/zheng/my_project
# set $PROJECT_ROOT = "/apollo/"

# GDB 搜索源码时优先搜索该目录
set directories /apollo

# 如果可执行文件里记录的路径与本机不一致：
# 例如构建机路径是 /build/agent/project_xxx/src
# 本地是 /home/zheng/project/src
# set substitute-path /build/agent/project_xxx $PROJECT_ROOT

###############################################
#  常用别名（提高效率）
###############################################

# 简易别名命令
define bt5
    backtrace 5
end
document bt5
    打印前 5 层调用栈
end

define regs
    info registers
end

define ths
    info threads
end

# 打印当前行全面信息
define here
    info source
    list
end

###############################################
# 加载符号时自动展开路径
###############################################
set substitute-path /usr/src/debug $PROJECT_ROOT

###############################################
#  断点增强
###############################################

# 捕获 C++ 异常（仅适用 libstdc++）
# catch throw
# catch catch

# 打印分段错误时的内存访问地址
set print address on

###############################################
#  调试体验增强插件（可选）
###############################################

# 启用 pwndbg/peda/mygdb 调试扩展（如果你装了）
# source ~/.gdb/peda/peda.py
# source ~/.gdb/pwndbg/gdbinit.py

###############################################
#  语言环境
###############################################

# 显示中文字符串
set charset UTF-8

# b lane_change_path.cc:88
# b lane_change_path.cc:91
# b lane_change_path.cc:96
# b lane_change_path.cc:195
# b lane_change_path.cc:162

# b path_optimizer_util.cc:169
# b piecewise_jerk_problem.cc:94

