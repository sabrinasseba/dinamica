# ROS Workspace 💻
<p align="justify">

Este repositorio conta com container docker com ROS Humble, cujo é destinado para trabalho de simulação de um atuador soldador para a matéria de Dinâmica de Sistemas Robóticos.
</p>

<p align="justify">

O que está **contido** neste repositório?
* Dockerfiles para algumas distribuições ROS com as instruções de construção necessárias.
* Scripts que tornam o docker um pouco mais fácil.
</p>

<p align="justify">

## Passo 1 - Clone este repositório

<p align="justify">

Para baixar o repositório, execute o comando abaixo. Todavia, a permissão de push na main está bloqueada, logo indicamos que execute um fork do repositório para o seu repositório. O botam fica no começo da página. 
</p>

```bash
git clone https://github.com/sabrinasseba/dinamica.git
```
## Passo 2 - ROS Humble Workspace 

<p align="justify">

O comando a seguir construirá a imagem docker necessária com imagem humble com alguns pacotes.
</p>

*Dentro da pasta "dinamica" no seu computador

```bash
docker/scripts/build.sh 
```

## Passo 3 - Versão completa do ROS Humble

<p align="justify">

O comando baixo, via o script **run.sh**, execuratá a imagem desejada e iniciará o container com o ROS funcional, pronto para ser utilizado. 
</p>

```bash
docker/scripts/run.sh
```
<p align="justify">

Em seguida, é necessário constroir o workspace, basta executar os comandos abaixo. 

```
colcon build
```
```
source install/setup.bash
```

Para executar a simulação do ROS, é necessário entrar na pacote **robot_description**, para isso, digite o comando abaixo.
</p>

```
cd robot_description
```
<p align="justify">

Assim, você estará na pacote que contém os pastas **config**, **launch**, **meshes**, **rviz2** e **urdf**. Dentro da pasta é possível analisar a construção dos arquivos xacro/urdf, launch files e as meshes utilizadas.
</p>

## Passo 5 - Executando a simulação

<p align="justify">
  
Para rodar a simualação do atuador no Rviz, basta ir para seu diretorio incial e utilizar o comando abaixo.

```
ros2 launch robot_description display.launch.py
```







