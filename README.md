## preinstall

```shell
getent group input 
sudo usermod -aG input $USER 
reboot
```

## debug

### bluetooth not working

```shell
sudo modprobe -r btusb
sudo modprobe -r btintel
sudo modprobe btusb
sudo modprobe btintel
```