// definir action nodes 
//Definir uma posição dos cones —> várias variáveis a partir de uma (centro, 4 quinas, meio da aresta de trás e da frente)
//variável da orientação da vaga (na direção Z apenas, pq o plano é XY)
    //pega a orientação Z do twist


//subscription da posição do carro e dos cones em tempo real

//etapa 1 - verifica se está na posição certa para começar
//- ré com volante para direita até (o a distância do centro parar de diminuir)
//- ré com volante para esquerda até (distância até centro da aresta_traseira ser pequena OU orientação do carro ser o suficiente (ja está reto))
//    - se ja estava reto, deixa volante reto e vai pra fente até a (dist to center parar de diminuir)
//    - se parou pq chegou mt próximo da aresta de tras, vira volante para direita e acelera até ficar reto ou chegar mt perto da borda
//        - se ficou reto, ai só vai pra frente ou p tras até a (dist to center ficar a minima)
//        - se não ficou reto, provavelmente não da pra estacionar… mas aí da pra tentar uma rotina que chega ainda mais próximo do cone

//InvPosicaoInicial --> o inverso de posicao inicial (retorna sucesso se NÃO estiver la)
    //posicao inicial é aquela paralela à vaga pra frente




//Etapa2 --> vai dá a ré (RUNNING) com volante para esquerda até ficar paralelo à vaga (pegar a orientação)
    //retorna FAILURE se chegar a uma distância suaficientemente próxima ao centro da aresta central traseira da vaga
    // retorna SUCCESS se conseguir ficar paralelo à vaga

//Etapa3 (fora do fallback da 2)--> vai pra frente com volante reto (RUNNING) até a dist to centro ficar mnínima
    //dai retorna SUCCESS, sai de tudo e para o veículo

//Etapa4 --> (mesmo fallback da 2) vira volante para direita e acelera até ficar paralelo
    //retorna success se conseguir --> vai pra etapa 3 (ajustando)
    //retorna failure se não conseguir (não pensei numa etapa 5, melhor parar tudo que ja era)


//Etapa1 --> ré com o volante para direita (ou uma variável, qualquer coisa só inverter os valores para esquerda)
    //Running até a distancia do centro do carro até o centro da vaga parar de diminuir
class Etapa1 : public BT::SyncActionNode
{
public:
    Etapa1(const std::string& name) :
        BT::SyncActionNode(name, {})
    {}

    BT::NodeStatus tick() override
    {
        std::cout << "Etapa1: " << this->name() << std::endl;
        //escreve no tópico do carla para virar o volante para esquerda (steer no máximo 1.22)
        //escreve no tópico do carla para dar ré (thorttle negativo -0.4 por ai)
        return BT::NodeStatus::RUNNING;
        //recebe do tópico de 


    }
}