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

Tambien puedes probar otras aplicaciones como el simulador gazebo:
```
ign gazebo
```


#### Nueva terminal al container

Para abrir una nueva terminal vinculada al container ejecuta

```
./docker/nueva_terminal.sh
```

#### Utilidad para trabajar mas facil!

Se pueden crear unos aliases para que correr el container o unirse al container sea mas rapido. Es decir, en vez de tener que hacer `cd <path-a-carpeta-workshop>` y luego el `./docker/run.sh` o `./docker/nueva_terminal.sh`, simplemente ejecutas un comando y listo:

 - Alias para lanzar el container:
    Primero muevete a la carpeta del repositorio `ros2_workshop`
    ```
    cd ros_workshop
    ```
    Crea el alias para correr el container, usa `--use_nvidia` o no en el alias dependiendo tu computadora.
    ```
    echo "alias ros2_workshop_run='cd $(pwd) && ./docker/run.sh --use_nvidia'" >> /home/$USER/.bashrc
    ```
 - Alias para unirse al container:
    Crea el alias para unirse al container con una terminal nueva cuando ya tenemos corriendo el container
    ```
    echo "alias ros2_workshop_join='cd $(pwd) && ./docker/nueva_terminal.sh'" >> /home/$USER/.bashrc
    ```

  Listo. Ahora abre una nueva terminal (tiene que ser nueva para que se carguen los aliases).

  Para para correr el container ejecuta:
  ```
  ros2_workshop_run
  ```
  Para unirse al container que esta corriendo ejecuta:
  ```
  ros2_workshop_join
  ```

  _Nota: Los aliases fueron agregados en el archivo `.bashrc` ubicado en `/home/$USER/.bashrc`. Para removerlos simplemente abre el archivo y elimina esas lineas agregadas al final._
