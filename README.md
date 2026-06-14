# A_estrela-no-Gazebo
Implementação do A* em um robô no Gazebo, fazendo sair de um labirinto

##  Estrutura

```
ros2_workshop/
├── Dockerfile
├── workshop_assets/
│   └── assets/
│       ├── launch/
│       ├── models/
│       ├── scripts/
│       ├── src/
|       |-- world/
│       ├── package.xml
│       └── CMakeLists.txt
```

---

## 🚀 Como Rodar (Ubunto 22.04)

###  1. Importe os arquivos

Abra um terminal, siga os passo de 1. a 9. no Github abaixo:
```
 https://github.com/milenafariap/ros2_workshop/
```
Após isso, pressione Ctrl + C no terminal, feche o terminal e finalize baixando os arquivos aqui presente;

###  2. Substitua os arquivos:

Substitua o explore_world.sdf na pasta world pelo baixado no passo anterior.

Logo após isso, mova os outros arquivos baixados para pasta scripts.

### 3.  Execute o programa

Abra dois terminais. Em um dos terminais, siga os passos do 6. ao 9. no link abaixo:
```bash
 https://github.com/milenafariap/ros2_workshop/
```
Aperte o "play" do Gazebo.
No outro terminal, execute isso:
```bash
 docker exec -it ros2_workshop_container bash
```
e logo após, no mesmo terminal, faça isso:
```bash
cd /root/workshop_assets
source install/setup.bash
cd assets/scripts
PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH python3 movimentacao_carrin.py
```

Se tudo for exceutado da maneira adequada, o carrinho dentro do Gazebo irá sair do labirinto de maneira autônoma.

---
