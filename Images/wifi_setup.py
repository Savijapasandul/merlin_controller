import secrets
data=None
with open('Images/input.yaml','r',encoding='utf-8') as file:
    data = file.readlines()
teamname=input("Enter Your Team name: ")
hotspotname='robot_'+teamname
passwd=teamname+'_'
passwd+=''.join(secrets.choice('BDGHJKLMNQR') for i in range(1,6))
data[16]=f'               "{hotspotname}":\n'
data[17]=f'                   password: "{passwd}"\n'
print(f'Create a hotspot with name {hotspotname}')
print(f'The password to be used is {passwd}')
input(f'Create a hotspot with name {hotspotname}.\n The password to be used is {passwd}.\n They are case-sensitive. Press enter to confirm you have noted this.')
with open('Images/50-cloud-init.yaml', 'w', encoding='utf-8') as file:
    file.writelines(data)
import os 
# os.system("cp Images/50-cloud-init.yaml 50-cloud-init.yaml")
if os.path.exists('/etc/netplan/50-cloud-init.yaml'):
    os.system("cp /etc/netplan/50-cloud-init.yaml Images/50-cloud-init-old.yaml")
    os.system("echo ubuntu123 | sudo -S mv Images/50-cloud-init.yaml /etc/netplan/50-cloud-init.yaml")
if os.path.exists('/etc/netplan/50-cloud-init.yaml'):
    os.system("cp /etc/netplan/50-cloud-init.yaml Images/50-cloud-init-new.yaml")
if os.path.exists('/etc/netplan/50-cloud-init.yaml'):
    os.system("sudo netplan --debug apply")