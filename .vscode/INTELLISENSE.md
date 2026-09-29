# Cursor / VS Code 中 C 工程 IntelliSense（F12）

本目录配置与 Keil 工程 `MDK-ARM/fY_270.uvprojx` 的宏、头文件路径对齐，用于编辑器内「转到定义」（F12）。

## 首次使用（必读）

### 0. 关于「Workspace Trust」（Cursor 默认没有这条命令）

**Cursor 默认关闭** VS Code 的 Workspace Trust（`security.workspace.trust.enabled` 默认为 false），所以命令面板里**搜不到** `Manage Workspace Trust` 是正常的，一般**不是** F12 无效的原因。

只有你在**用户 settings.json** 里手动加了 `"security.workspace.trust.enabled": true` 之后，才会出现信任相关命令；且左下角出现 **Restricted Mode** 时才需要去信任文件夹。

### 1. 语言服务（clangd）

Cursor 2.x 的 **anysphere.cpptools** 通过扩展包里的 **clangd** 提供 F12/悬停。本工程已配置 `compile_commands.json`，且 **`clangd.enable": true`**。

2. **Cursor 里多种搜索框**（不要混）：
   - **Ctrl+Shift+P** → **Search actions…**（Actions）
   - **Ctrl+Alt+P** → **Search agents…**（Agents；列表里的 “Manage Workspace Trust” 是 **Agent 名称**，不是系统信任设置）
   - **F1** 或 **Ctrl+Shift+Alt+P** → **Show All Commands**（扩展命令：`clangd: Download language server`）
   - **Ctrl+P** → 打开文件；在框内先输入 **`>`** 再输入命令名（等价命令面板）
   - **Cursor 精简 View 菜单里没有「命令面板」** 时，用 **Ctrl+P → 输入 `>` → 再输入命令**（最可靠）
   - **注意**：View → Settings（**Ctrl+,**）在 Cursor Glass 里常打开 **General（账号/隐私）**，**搜不到 Extensions**。扩展与 clangd 请用下面方式。
   - **Ctrl+Shift+X** → 扩展市场（或 **Ctrl+P** → **`>`** → 输入 **Extensions: Install Extensions**）
   - **Ctrl+P** → **`>`** → **Preferences: Open User Settings (JSON)** 编辑 `clangd` / `C_Cpp` 配置
   - 查看快捷键：**Ctrl+K Ctrl+S**，搜索 **Show All Commands**
3. 工作区根目录须为 **`cursor_fy`**（与 Keil 的 `../UserFiles` 相对路径一致）。
3. 首次打开 `.c` 文件时，若提示安装 **clangd**，请点 **Download / 安装**。
4. `Ctrl+Shift+P` → **clangd: Restart language server**，等待索引后再试悬停 / F12。

## 仍不能跳转时请检查

1. **工作区根目录**必须是 `cursor_fy`（能看到 `compile_commands.json` 和 `MDK-ARM`），不要只打开 `Core` 子文件夹。
2. 输出面板 → 下拉选 **C/C++** / **C/C++ Diagnostics**，看是否有 “Cannot find compile_commands” 或配置未激活。
3. 若改回只用 **anysphere.cpptools**，须把 `"C_Cpp.intelliSenseEngine"` 改回 `"default"`，并**禁用 clangd** 扩展，否则会再次冲突。
4. 确认扩展 **anysphere.cpptools** 已启用（扩展页不要 Disabled）。
5. 试 **右键 → Go to Definition**，或 **F12**；对 `HAL_Init` 先测 HAL 是否能跳，再测 `APP_Ctrl_System_Init`。

## 验收示例

- `Core/Src/main.c` 中 `APP_Ctrl_System_Init` → 应跳转到 `UserFiles/APP/app/app_ctrl.c`。
- `HAL_Init` → 应进入 STM32 HAL 头文件/源文件。

## 无法 F12 进入实现的情况（正常）

以下符号仅有**预编译库**或**头文件声明**，没有随仓库提供的 `.c` 源码，F12 最多到声明，不能进函数体：

| 组件 | 库 / 头文件 |
|------|-------------|
| 地理引导算法 | `UserFiles/APP/GEO_Track_C.lib`、`GEO_Track_C.h` |
| 伺服库 | `UserFiles/APP/servo/Servolib.lib`、`SFlibhead.h` |

实际固件仍由 **Keil MDK** 链接上述 `.lib` 编译；本配置**不影响** Keil 构建。
