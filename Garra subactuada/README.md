# underactuated-gripper

En este repositorio se encuentran todos los ficheros relativos a la garra subactuada de cuatro dedos con eslabón elástico. El repositorio está organizado de la siguiente forma:

- **Carpeta arduino.** Contiene los codigos fuente de los dispositivos implicados en la garra. Contiene los scripts del sensor de fuerzas cilíndrico rojo basado en galgas extensiométricas y el sensor HX711 (Lectura\_hx711), y de control de la garra (lecturaMotorEncoder\_V8).

- **Carpeta Data.** Contiene los datos de los experimentos realizados con la garra. Los datos se encuentran separados por carpetas, atendiendo al tipo de experimento con el que se han obtenido.

- **Carpeta grippers\_conf.** Contiene los parametros de las garras subactuada de cuatro dedos y rolling fingers en formato JSON. Se pueden cargar desde la interfaz del robot para hacer el cambio de efector final. 

- **Carpeta ros.** Contiene el paquete ROS (version melodic) desarrollado para el control y la monitorización de la garra desde dicho OS. 

- **Carpeta scripts.** Contiene los programas de matlab para el computo del modelo kinetostatico de los dedos, así como los scripts para el procesamiento de los datos de los experimentos y su representación gráfica.
