# VSS
Este repositório servirá para manter os códigos que serão utilizados em minha Iniciação Científica/matéria de robótica feita com o professor [Eduardo Todt](http://www.inf.ufpr.br/todt/).
## Objetivo:
O objetivo inicial é de desenvolver um novo firmware (baseado no anteriormente desenvolvido por Alexandre Calerio) para o robô da categoria de [IEEE Very Small Size Soccer](http://www.cbrobotica.org/?page_id=81).

## Bibliotecas adicionais:
### Arduino:
Todas as bibliotecas podem ser obtidas por meio do próprio menu do software do Arduino  (Sketch -> Incluir Biblioteca -> Gerenciar Bibliotecas).

A instalação do Motorino.h(atualmente não utilizado) e demais informações estão demonstradas no trabalho feito pelo Alexandre(vide códigos-base).

#### Breve explicação destas bibliotecas:
- [RF24.h](https://github.com/maniacbug/RF24) e [RF24Network.h](http://maniacbug.github.io/RF24Network/index.html) servem para que a comunicação sem-fio (via rádio) aconteça entre as placas
- [SPI.h](https://www.arduino.cc/en/reference/SPI) serve para o uso de protocolos de microcontroladores síncronos, assim como para que aconteça a conexão Serial

### Python:
Todas as bibliotecas são instaladas com o uso do pip, que normalmente já vem instalado junto com o prório Python. Mas caso você não tenha, [veja este guia](https://pip.pypa.io/en/stable/installing/).

- [py-getch](https://pypi.org/project/getch/)(usado na primeira versão do robô):
Basimente usado para a função de para pegar o input do usuário sem a necessidade de teclar o enter.
Pode ser instalado com o seguinte comando:
```
python -m pip install py-getch
```

- [py-serial](https://pythonhosted.org/pyserial/):
Utilizado para que ocorra a conexão via serial entre o programa Python e (no caso) a base transmissora de dados.
```
python -m pip install pyserial
```
**Cuidado: não** instale a biblioteca 'serial' no lugar da 'pyserial'. O programa **não fuicionará** com esta outra biblioteca.


## Uso:
Atualmente, o robô foi desenvolvido com a seguinte ideia:

- Equipes próprias para a captura do campo e cálculo dos movimentos dos robôs enviarão mensagens de comando com o padrão de mensagem apresentado [neste documento](https://docs.google.com/document/d/12rgDv1aShHZEZKDBIDaRbKfi_514n4bRbpjx6Cca2Ug/edit?usp=sharing) via entrada padrão do Terminal. 
- Estes comandos serão lidos e repassados para comunicação Serial (para a qual a base está conectada) com o uso do programa **interface.py**.
- A **base** fará a tradução dos comandos para o tipo de mensagem utilizado para a comunicação entre os robôs. Após isso, a própria base enviará estas mensagens via rádio para os robôs.
- Os **robôs** receberão as mensagens e, caso o número de identificação se refira ao número de um deles, o comando será realizado.