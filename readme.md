# Readme del Proyecto del segundo Parcial de Sistemas Operativos

##Componentes del Proyecto

Este sistema se compone de 8 archivos:

* proyecto.c => archivo donde se encuentra el codigo de todo el sistema
* sensorSO1.c => archivo que contiene el codigo que simula el sensor central del auto (puede cambiar)
* sensorSO2.c => archivo que contiene el codigo que simula el sensor lateral del auto
* sensorSO3.c => archivo que contiene el codigo que simula el sensor lateral del auto
* Makefile => archivo que sirve para compilar el proyecto completo
* launcher.sh => archivo con instrucciones para ejecutar los 3 simuladores al mismo tiempo
* config.cfg => archivo que contiene las configuraciones iniciales que van a ser cargadas al sistema
* changes.cfg => archivo en donde se escriben los valores que se van a cambiar en I,Q,W,T

## Configuracion del proyecto 

Para ejecutar el sistema, ejecute el comando make en la consola del SO, esta instruccion
tomara las configuraciones del archivo Makefile y compilara todos los archivos.

Los archivos que se generan luego de la compilacion son los siguientes:

* proyecto => ejecutable del sistema
* sensorSO1 => ejecutable que contiene el codigo que simula el sensor central del auto (puede cambiar)
* sensorSO2 => ejecutable que contiene el codigo que simula el sensor lateral del auto
* sensorSO3 => ejecutable que contiene el codigo que simula el sensor lateral del auto

En caso de que tenga problemas con el archivo makefile, puede ejecutar los siguientes comandos en la consola del SO:

	gcc -o sensorSO1 sensorSO1.c -lm
    gcc -o sensorSO2 sensorSO2.c -lm
    gcc -o sensorSO3 sensorSO3.c -lm
    gcc -o proyecto proyecto.c -lm -lpthread

Luego, si desea cambiar las configuraciones iniciales en el archivo config.cfg, debe ingresar los datos en el siguiente formato

idmem_sensorLaser1|idmem_sensorGiro1|....|idmem_sensorLaserN|idmem_sensorGiroN,I,Q

## Ejecucion del sistema
Para iniciar el sistema, debera inicializar los 3 simuladores de sensores. Esto lo puede hacer ejecutando el archivo launcher.sh, el cual contiene las instrucciones para inicializarlos al mismo tiempo.

    ./launcher.sh

Luego debe ejecutar en la consola el archivo ejecutable del proyecto
pasando como argumento el archivo de configuraciones:

    ./proyecto "config.cfg"

El sistema verificara que haya ingresado los datos en el formato solicitado, caso contrario
se mostrara un mensaje de error y se finalizara el sistema.

Si desea, es posible cambiar los valores de distancias de los sensores, abriendo el archivo fuente de alguno de ellos y cambiar el macro DISTANCIA al valor deseado. Recuerde que tendra que volver a ejecutar make para que se generen los nuevos ejecutables con la nueva configuracion.

## Modificacion de variables de muestreo
Para modificar las variables I,Q,T,W del sistema, debera ingresar los datos que desea cambiar en el archivo changes.cfg, puede ingresarlos utilizando el comando echo en la consola:

    echo [variables a modificar] > changes.cfg

El formato para modificar las variables se muestra a continuacion:

    var_name1=val1,var_name2=val2,......var_namen=valn

Ejemplo:

    echo I=10,Q=15 > changes.cfg
    echo I=7,Q=10,W=3.5,T=0.2 > changes.cfg

El sistema verificara que los datos fueron ingresados en el formato solicitado. Los cambios
en I,W,T se daran inmediatamente, pero el de Q surtira efecto inmediatamente despues de que el muestreo actual termine, caso contrario tambien se aplicara inmediatamente.


##Limitaciones del proyecto

Este proyecto cuenta con las siguientes limitantes:

* El formato de configuraciones iniciales debe seguir estrictamente el orden: 
        
        idLaser|idGyroscopio|idLaser|idGyroscopio
    
  Es decir que se debe colocar el id de memoria de un Laser junto con el de su giroscopio correspondiente, si esto se invierte o se especifica de otra manera es posible que se tenga comportamientos o resultados no previstos

* El sistema asume que el primer id de memoria de Laser en el archivo de configuraciones es el del Laser Central

* Dado el siguiente enunciado : "si alguno de los lasers difiere en W * sigma de cualquier otro laser entonces el objeto es un Obstaculo", La interpretacion que se ha seguido en este sistema del enunciado es:

        (promLi - promLj) == W * sigma
 
Lo cual resulta un caso poco probable. Si la relacion fuera >= entonces hay una probabilidad mas grande de caer en esos valores, sin embargo, se estipulo trabajar exactamente a la literatura del problema.