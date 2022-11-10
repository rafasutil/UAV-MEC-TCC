# UAV-MEC

Esse código contém modificações que fiz no [projeto original](https://github.com/YouhuiGan/UAV_DDPG) para desenvolver meu trabalho de graduação.

Existem quatro arquivos com a modelagem do sistema:
* SystemModel_all_improvs.py: Contém as duas melhorias implementadas juntas.
* SystemModel_DEFAULT.py: Contém o código original.
* SystemModel_improv1.py: Contém apenas a melhoria do canal de downlink.
* SystemModel_improv2.py: Contém a melhoria das probabilidades de falha

Para executar o código antes acesse o [repositório original](https://github.com/YouhuiGan/UAV_DDPG) que contém algumas instruções.

Você deve escolher um dos modelos acima e renomeá-lo para *SystemModel.py*, que será o código usado na execução.