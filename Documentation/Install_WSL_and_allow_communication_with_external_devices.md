# A Guide to Configuring WSL2 for ROS2 on Windows 11

## 1. Install Windows Terminal
First, I recommend installing **Windows Terminal** if you haven’t already.

[Install Windows Terminal](https://aka.ms/terminal)

## 2. Install WSL2
Open PowerShell as an **Administrator** (right-click and select `Run as administrator`), enter the following command, wait for the installation to finish, and then **restart your machine**.

```powershell
wsl --install
```

WSL installs the latest Long-Term Support (LTS) version of Ubuntu by default, which is ideal for a ROS2 development environment. If the installation fails to include Ubuntu, you can install it manually using this command:

```powershell
wsl --install -d Ubuntu-24.04
```

### 2.1 Post-Installation
Open Windows Terminal, click the arrow on the "New Tab" icon, and select **Ubuntu**. You will be prompted to create a UNIX username and password.

> [!NOTE]
> You will not see characters appearing in the terminal while typing your password. This is standard security behavior.

Once the user is created, you will be directed to your Ubuntu home directory. This is indicated by `your-username@your-wsl-machine-name:~`, where the tilde (`~`) represents your home folder path. Update the Ubuntu installation by running the following command:

```bash
sudo apt update && sudo apt upgrade -y
```

### 2.2 Enable GUI Applications
To run Linux GUI applications with hardware-accelerated OpenGL rendering, ensure you have the latest GPU drivers for your specific system:

* [**Intel** GPU driver](https://www.intel.com/content/www/us/en/download/19344/intel-graphics-windows-dch-drivers.html)
* [**AMD** GPU driver](https://www.amd.com/en/support)
* [**NVIDIA** GPU driver](https://www.nvidia.com/Download/index.aspx?lang=en-us)

## 3. Install ROS2
With WSL up and running, follow the official ROS2 documentation for **Ubuntu (deb packages)**. [ROS 2 Documentation: Jazzy Installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

> [!IMPORTANT]
> Follow the installation guide carefully. Ensure you install `ros-jazzy-desktop` and **avoid** the `ros-jazzy-base` command mentioned in the official guide.

### 3.1 Install Gazebo and Other Dependencies
To install a compatible version of the Gazebo simulator, refer to the [Gazebo Harmonic Binary Installation](https://gazebosim.org/docs/harmonic/install_ubuntu/).

* **Compatibility:** Review the [ROS 2 and Gazebo pairing documentation](https://gazebosim.org/docs/harmonic/ros_installation/) for version details.
* **Other Dependencies:** Additional necessary dependencies are listed in the main [README.md of this repository](https://github.com/dsosa114/movilidad_inteligente).

## 4. WSL2 Networking
Reliable networking typically requires **Windows 11 Pro** to utilize Hyper-V for creating a virtual network switch. If you are using Windows 11 Home, skip to **Section 4.2** for a mirrored-mode workaround.

---

### 4.1 Hyper-V Configuration (Windows 11 Pro Only)

#### 4.1.1 Enable Hyper-V
Check if Hyper-V is enabled by running this command in an Administrator PowerShell prompt:

```powershell
Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V-All
```

If it is disabled, enable it with the following command and restart your computer:

```powershell
Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Hyper-V -All
```

#### 4.1.2 Create the Virtual Network Adapter
Run this command to list your physical adapters:

```powershell
Get-NetAdapter
```

Identify your active **Ethernet** or **Wi-Fi** adapter. To create the switch, run the command below, replacing `"WiFi"` with the exact name of your adapter if it differs:

```powershell
New-VMSwitch -Name "External Switch" -NetAdapterName "WiFi" -AllowManagementOS $true
```

#### 4.1.3 Configure WSL2 Settings
We will now configure WSL2 to use the new adapter and allocate appropriate system resources. Open the configuration file in Notepad:

```powershell
notepad .wslconfig
```

If you have **32GB of RAM or more**, use the following configuration:

```ini
[wsl2]
swap=8589934592
memory=25769803776
networkingMode=bridged
vmSwitch="External Switch"
```

**Memory Adjustments:**
* **16GB+ RAM:** Set `memory=12884901888`
* **8GB+ RAM:** Set `memory=6442450944`

> [!WARNING]
> You can allocate less memory, but you may encounter errors when building large ROS2 packages. If you experience memory issues with `colcon`, try increasing the swap to `swap=17179869184`.

**File Placement:** This file must be saved in your **User Profile** folder. You can quickly access this by typing `%USERPROFILE%` in the File Explorer address bar. 

> [!IMPORTANT]
> Please ensure that the extension of the newly created file is `.wslconfig` and not another extension like `.txt`. You can check this in the properties of the file.

---

### 4.2 Workaround for Windows 11 Home
If Hyper-V is unavailable, you can **mirror** the Windows network adapter. This allows the WSL2 installation to communicate with other devices on your Local Area Network (LAN).

Open the configuration file:
```powershell
notepad .wslconfig
```

Remove any `vmSwitch` lines and set the networking mode as follows:

```ini
[wsl2]
networkingMode=mirrored
```

Next, allow inbound connections to detect ROS2 nodes on other machines by running this command in an Administrator PowerShell:

```powershell
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

## Tips and "Nice to Haves"
* **File Access:** To open a WSL folder in Windows Explorer, type `explorer .` inside the Linux terminal.
* **VS Code:** For a seamless development experience, [set up VS Code for WSL](https://learn.microsoft.com/en-us/windows/wsl/tutorials/wsl-vscode).

## Troubleshooting
* **ROS2 Node Issues:** If `ros2 node list` fails, the daemon might not be running. Start it with:
    ```bash
    ros2 daemon start
    ```
* **Firewall Blocking:** If external nodes are not detected, run these commands in Administrator PowerShell to open the necessary ROS2 ports, then reboot:
    ```powershell
    New-NetFirewallRule -DisplayName "ROS2 UDP 7400-7600" -Direction Inbound -Action Allow -Protocol UDP -LocalPort 7400-7600
    New-NetFirewallRule -DisplayName "ROS2 UDP 7400-7600 Outbound" -Direction Outbound -Action Allow -Protocol UDP -LocalPort 7400-7600
    ```
* **Reverting to Default:** To return to standard local networking, change `networkingMode=mirrored` to `networkingMode=NAT` in your `.wslconfig` file.

---

## References

[Install WSL](https://learn.microsoft.com/en-us/windows/wsl/install)

[Accessing network applications with WSL](https://learn.microsoft.com/en-us/windows/wsl/networking)

[Set up a WSL development environment](https://learn.microsoft.com/en-us/windows/wsl/setup/environment#set-up-your-linux-username-and-password)

[Get started using VS Code with WSL](https://learn.microsoft.com/en-us/windows/wsl/tutorials/wsl-vscode)

[Ubuntu (deb packages) — ROS 2 Documentation: Jazzy documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

[Docs / Gazebo Harmonic — Binary Installation on Ubuntu](https://gazebosim.org/docs/harmonic/install_ubuntu/)

[Docs / Gazebo Harmonic — Installing Gazebo with ROS](https://gazebosim.org/docs/harmonic/ros_installation/)

[Github — Movilidad inteligente](https://github.com/dsosa114/movilidad_inteligente)