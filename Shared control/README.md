# Requisitos del Shared control 

Para el funcionamiento del shared control es necesario el uso de la librería libfranka y un sistema operativo de tiempo real (en la página web de franka emika se explica como instalarlo, añadir enlace al tutorial), un robot franka emika (he implementado el control directamente sobre la API de este robot), el paquete matlogger para recopilación de datos (añadir link al repo de matlogger).

La carpeta franka\_ros contiene el paquete de ros de franka emika con todos los controladores que he desarrollado (es la versión del robot FR3), mas concretamente es dentro de la carpeta "franka\_example\_controllers" donde se encuentran todos los controladores que he desarrollado (a parte de los que vienen en el paquete por defecto). Es importante que el paquete "subact\_gripper" se encuentre en el mismo workspace de ros que el paquete "franka\_ros", ya que se requiere el uso de la garra en algunos controladores. 

