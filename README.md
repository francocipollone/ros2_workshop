# ros2_workshop

## Links utiles!

 - https://docs.ros.org/en/humble/index.html
 - https://github.com/Ekumen-OS/andino
 - https://github.com/Ekumen-OS/andino_gz

## Preparacion del environment

Un Docker es provisto para correcto setup del environment.
En caso que se quiera utilizar la host machine para el workshop(Y no utilizar containers) entonces solo Ubuntu 22.04 / ROS 2 Humble es soportado.

### Docker

#### Prerequisitos

Es un requerimiento tener `docker engine` instalado en la host machine.

* [Guia de instalacion de Docker](https://docs.docker.com/engine/install/ubuntu/) y [pasos post-instalacion](https://docs.docker.com/engine/install/linux-postinstall/)
  * Sigue los pasos 1, 2 y 3 indicados en [`Install using the apt repository`](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository)

  * Tambien, sigue los pasos 1, 2, 3 y 4 en [`Manage Docker as a non-root user`](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user)

Para tener soporte de GPU NVidia, `nvidia-container-toolkit` tiene que ser instalado.
**Salta este paso si no tienes una tarjeta grafica NVidia**
  * Primero asegurate que tienes los drivers instalados:
    ```sh
    nvidia-smi
    ```
  * Luego sigue la [Guia de instalacion de NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
    * Siga los pasos 1, 2 y 3 indicados en [`Installing with Apt`](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-apt)

#### Clona el repositorio


git clone https://github.com/francocipollone/ros2_workshop.git --recursive


#### Compila la imagen
```
cd ros2_workshop
./docker/build.sh
```

#### Lanzar container

Si tenes tarjeta grafica NVidia:
```
./docker/run.sh --use_nvidia
```
Si NO tenes tarjeta grafica NVidia:
```
./docker/run.sh
```

Una vez adentro del container, para verificar que la visualizacion de aplicaciones GUI es correcta, verifica por ejemplo si la app rviz2 puede ser lanzada:
```
rviz2
```
Si una ventana se abre entonces esta funcionando!.


#### Nueva terminal al container

Para abrir una nueva terminal vinculada al container ejecuta

```
./docker/nueva_terminal.sh
```
