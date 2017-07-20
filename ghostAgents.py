# ghostAgents.py
# --------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from game import Agent
from game import Actions
from game import Directions
import random
from util import manhattanDistance as h
import util
from searchAgents import PositionSearchProblem

class AgenteFantasma(Agent):
    def __init__( self, index ):
        self.index = index # Indice do fantasma 


    def getAction(self, state):
        acoesLegais = state.getLegalActions(self.index)       # Retorna uma lista com posicoes legais
        estadoFantasma = state.getGhostState(self.index)      # Estado do fantasma

        comMedo = estadoFantasma.scaredTimer > 0              # Diz se o fantasma esta com medo. Se for maior que 1 entao ele esta com medo

        problema = PositionSearchProblem(state)               # Problema atual
        problema.startState = state.getGhostPosition(self.index) # Estado inicial do fantasma. Esse self indica que eh o fantasma dessa instancia
        start = problema.startState                              # O inicio eh o problema inicial
        problema.goal = state.getPacmanPosition() 
        objetivo = problema.goal                                 # O fantasma precisa alcancar o pacman, entao a posicao dele eh o objetivo do fantasma

        fantasmas = state.getGhostPositions()                    # Lista contendo as posicoes dos fantasmas
        fantasmas.remove(state.getGhostPosition(self.index))     # Remove a posicao dessa instancia

        visitados = fantasmas                                    # Utiliza as posicoes dos outros fantasmas como visitados. Isso foi feito para que os fantasmas nao peguem a mesma rota.
        borda = util.PriorityQueue();                            # Faz borda ser uma lista de prioridade
        borda.push(start, h(start, objetivo))                    # Coloca o inicio junto com o valor.
        antecessores = {}                                        # Dicioario para guardar as relacoes de um estado pai e filho. Usado para ter o caminho de volta quando encontrar o objetivo e esse for a melhor acao 
        g = {start : 0}                                          # g(x) = 0 no primeiro caso
        f = {start : h(start, objetivo)}                         # f(x) = g(x) + h(x). Como g(x) eh zero entao nao conta nesse caso

        estado_candidato = None                                  # Estado que pode ser aceito como o proximo

        while not borda.isEmpty():
            noAtual = borda.pop()
            visitados.append(noAtual)

            if problema.isGoalState(noAtual):
                estado_candidato = self.caminho(antecessores, noAtual)[-2] # Apos chegar no estamo objetivo, o fantasma precisa saber como chegar la. Retorna uma lista ao contrario
                break                                                      # Encontrou o caminho entao sai do loop

            for vizinho, acao, custoVizinho in problema.getSuccessors(noAtual): # problema.getSuccessors(noAtual) retorna uma tupla co tres elementos. estado, acao e custo
                if vizinho in visitados: # Se o estado da iteracao ja estiver em visitados entao ele nao pode pegar essa rota
                    continue

                if not borda.contains(vizinho): 
                    borda.push(vizinho, custoVizinho + h(vizinho, objetivo)) # Caso o vizinho nao esteja na borda entao coloca la

                custoAcumulado = g[noAtual] + custoVizinho # Busca A* tem o custo acumulado sem a heuristica 
                if custoAcumulado >= g.get(vizinho, float("inf")):
                    continue

                antecessores[vizinho] = noAtual # Guarda quem eh o pai desse no atual. Cada no e um estado com posicao valida 
                g[vizinho] = custoAcumulado # g(x) desse estado. Precisa ser acumulado por causa do metodo
                f[vizinho] = g[vizinho] + h(vizinho, objetivo) # f(x) = g(x) + h(x)


        melhorAcao = None

        for sucessor, acao, custo in problema.getSuccessors(start):# Vai selecionar a melhor acao seguindo as regras de acoes legais
            if sucessor == estado_candidato:
                if acao in acoesLegais:
                    melhorAcao = acao

        if melhorAcao == None: # Se nao tem nenuma acao melhor entao pega qualquer uma que seja legal
            melhorAcao = random.choice(acoesLegais)

        #Se o fantasma estiver assustado, entao ele ira para qualquer direcao que nao seja a melhor, a nao ser que a melhor seja a unica direcao possivel
        if comMedo and melhorAcao != None and len(acoesLegais) > 1: # Retira a melhor posicao para nao ir de encontro ao pacman
            acoesLegais.remove(melhorAcao)
            melhorAcao = random.choice(acoesLegais)
        elif melhorAcao == None:
            melhorAcao = random.choice(acoesLegais)
            
        return melhorAcao

    # Faz o caminho inverso dos nos que foram expandidos ate o no atual. Cada frame o fantasma busca o melhor caminho
    # mas ele precisa saber como voltar por ele. Esse def caminho resolve esse problema 
    def caminho(self,antecessores, noAtual):  
        
        caminho_total = [noAtual]
        
        while noAtual in antecessores.keys():
            noAtual = antecessores[noAtual]
            caminho_total.append(noAtual)
        
        return caminho_total










class GhostAgent( Agent ):
    def __init__( self, index ):
        self.index = index

    def getAction( self, state ):
        dist = self.getDistribution(state)
        if len(dist) == 0:
            return Directions.STOP
        else:
            return util.chooseFromDistribution( dist )

    def getDistribution(self, state):
        "Returns a Counter encoding a distribution over actions from the provided state."
        util.raiseNotDefined()

class RandomGhost( GhostAgent ):
    "A ghost that chooses a legal action uniformly at random."
    def getDistribution( self, state ):
        dist = util.Counter()
        for a in state.getLegalActions( self.index ): dist[a] = 1.0
        dist.normalize()
        return dist

class DirectionalGhost( GhostAgent ):
    "A ghost that prefers to rush Pacman, or flee when scared."
    def __init__( self, index, prob_attack=0.8, prob_scaredFlee=0.8 ):
        self.index = index
        self.prob_attack = prob_attack
        self.prob_scaredFlee = prob_scaredFlee

    def getDistribution( self, state ):
        # Read variables from state
        ghostState = state.getGhostState( self.index )
        legalActions = state.getLegalActions( self.index )
        pos = state.getGhostPosition( self.index )
        isScared = ghostState.scaredTimer > 0

        speed = 1
        if isScared: speed = 0.5

        actionVectors = [Actions.directionToVector( a, speed ) for a in legalActions]
        newPositions = [( pos[0]+a[0], pos[1]+a[1] ) for a in actionVectors]
        pacmanPosition = state.getPacmanPosition()

        # Select best actions given the state
        distancesToPacman = [manhattanDistance( pos, pacmanPosition ) for pos in newPositions]
        if isScared:
            bestScore = max( distancesToPacman )
            bestProb = self.prob_scaredFlee
        else:
            bestScore = min( distancesToPacman )
            bestProb = self.prob_attack
        bestActions = [action for action, distance in zip( legalActions, distancesToPacman ) if distance == bestScore]

        # Construct distribution
        dist = util.Counter()
        for a in bestActions: dist[a] = bestProb / len(bestActions)
        for a in legalActions: dist[a] += ( 1-bestProb ) / len(legalActions)
        dist.normalize()
        return dist
