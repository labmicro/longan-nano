# Pruebas en la Longan Nano

Este repositorio tiene proyectos de prueba para ensayar el diseño de los diferentes dispositivos que se integraran en la placa educativa que se diseña para la asignatura [Sistemas con Microprocesadores y Microcontroladores (15_ECC)](http://microprocesadores.unt.edu.ar/procesadores) de la carrera de [Ingeniería en Computación](https://www.facet.unt.edu.ar/ingcomputacion/) de la [Facultad de Ciencias Exactas y Tecnología](https://www.facet.unt.edu.ar/) de la [Universidad Nacional de Tucumán](http://unt.edu.ar/).

## Empezando con el proyecto

Este repositorio utiliza un [submódulo GIT](https://git-scm.com/book/es/v2/Herramientas-de-Git-Submódulos) para el almacenar las refrencias al framework [Muju](https://github.com/labmicro/muju). Para clonar el repositorio, con GIT 2.13 o más nuevo, debe utilizarse el comando:

```
git clone --recurse-submodules https://github.com/labmicro/longan-nano.git
```

Si utiliza una versión anterior de GIT el comando será:

```
git clone --recursive https://github.com/labmicro/longan-nano.git
```

## Uso del repositorio

Este repositorio utiliza [pre-commit](https://pre-commit.com) para validaciones de formato. Para trabajar con el mismo usted debería tener instalado:

1. pre-commit (https://pre-commit.com/#install)

Después de clonar el repositorio usted debería ejecutar el siguiente comando:

```
pre-commit install
```

Para generar la documentación del proyecto se utiliza el siguiente comando:

```
make doc

```

Para compilar el proyecto se utiliza el siguiente comando:

```
make all

```

## License

This work is distributed under the terms of the [MIT](https://spdx.org/licenses/MIT.html) license.
