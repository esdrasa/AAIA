# -*- coding: utf-8 -*-
# pacmanAgents.py
# ---------------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

from pacman import Directions
from game import Agent
import random
import game
from util import manhattanDistance as h
import util
from searchAgents import PositionSearchProblem

# python pacman.py -g AgenteFantasma -p PacmanAgent

class PacmanAgent(game.Agent):
	def getAction( self, state ):
		acoesLegais = state.getLegalActions()		# Lista de acoes legais
		fantasmas = state.getGhostPositions()		# Lista de posicoes dos fantasmas

		problema = PositionSearchProblem(state)			# O problema atual
		#problema.startState = state.getPacmanPosition()			# Estado Inicial. Ja inicia com a posicao inicial do Pacman
		start = problema.getStartState()					# Estado inicial do problema

		
		problema.goal = min([(h(start, food), food) for food in state.getFood().asList()])[1]			#Objetivo e a comida mais proxima, infelizmente por enquanto ne

		visitados = set()						# Conjunto de estados ja visitados
		borda = util.PriorityQueue()					# Fila de prioridade contendo os elementos da borda
		antecessor = {}							# Dicionario com relacoes entre estados pai e filho
		g = {start: 0}						# g(x) e zero no inicio
		f = {start: h(start, problema.goal)}	# Primeiro f(x) conta so com a heuristica pois o g(x) e zero
		borda.push(start, f[start])				#A borda e uma pilha de estados ordenada por f(x)

		estado_candidato = None						#Estado candidato a ser suscetivel

		while not borda.isEmpty():					#A*
			noAtual = borda.pop()
			visitados.add(noAtual)
			#Se chegarmos ao estado final, recriamos um caminho de estados ate o inicio da busca, tirando o estado inicial
			if problema.isGoalState(noAtual):			
				estado_candidato = self.caminho(antecessor, noAtual)[-2]
				break

			#Expande o estado atual em ate 4 vizinhos ( Norte, Sul, Leste e Oeste )
			for vizinho, acao, custoVizinho in problema.getSuccessors(noAtual):
				
				comMedo = False
				
				#Verifica se ha fantasmas no caminho
				if vizinho in fantasmas:
					for index in xrange(1, state.getNumAgents()):
						if state.getGhostState(index).scaredTimer > 0 and state.getGhostPosition(index) == vizinho: # Verifica se o fantasma esta com medo e se a proxima posicao tem um fantasma
							comMedo = True

				if (vizinho in fantasmas and comMedo) or vizinho not in fantasmas:

					if vizinho in visitados:	# Verifica se o vizinho(estado) ja foi visitado
						continue

					if not borda.contains(vizinho): #Se este vizinho nao esta na borda
					
						f[vizinho] = h(vizinho, problema.goal)
						borda.push(vizinho, f[vizinho])

					
					custoAcumulado = g[noAtual] + custoVizinho
					#Verifica se ir para esse vizinho vale a pena
					if custoAcumulado >= g.get(vizinho, float("inf")):
						continue

					
					antecessor[vizinho] = noAtual		# Guarda quem é o pai desse vizinho para poder fazer o caminho inverso depois
					g[vizinho] = custoAcumulado	#g(x) desse vizinho
					#f(x) = g(x) + h(x)
					f[vizinho] = g[vizinho] + h(vizinho, problema.goal)

		melhorAcao = None

		#Retorna a melhor acao legal
		for sucessor, acao, custo in problema.getSuccessors(start):
			if estado_candidato == sucessor:
				if acao in acoesLegais:
					melhorAcao = acao
		# Se nao for a melhor acao entao retorna uma acao aleatoria dentre as legais
		if melhorAcao == None:
			melhorAcao = random.choice(acoesLegais)

		return melhorAcao



	#Recria o caminho de nos expandidos ate o no atual
	def caminho(self, antecessor, noAtual):
		caminhoTotal = [noAtual]
		while noAtual in antecessor.keys():
			noAtual = antecessor[noAtual]
			caminhoTotal.append(noAtual)
		return caminhoTotal


"""class aStarPacmanAgent(game.Agent):
	def getAction( self, state ):
		legalActions = state.getLegalActions()		#Lista de ações legais
		ghosts = state.getGhostPositions()
		
		STATE = 0							#Para a tupla de getSuccessors
		ACTION = 1							#Para a tupla de getSuccessors
		COST = 2							#Para a tupla de getSuccessors
#		MAX_DEPTH = 15


		problem = SA.PositionSearchProblem(state)			#O problema atual
		problem.startState = state.getPacmanPosition()			#Estado Inicial. Acho que ele já faz isso no construtor de searchAgents. Pega a posição do Pacman
		start = problem.getStartState()					#O início é o estado inicial do problema

		
		problem.goal = min([(util.manhattanDistance(start, food), food) for food in state.getFood().asList()])[1]			#Objetivo é a comida mais próxima, infelizmente por enquanto né

		closedSet = set()						#ClosedSet é o conjunto de estados já visitados
		openSet = util.PriorityQueue()					#OpenSet é a borda
		cameFrom = {}							#Dicionário com relações entre estados pai e filho
		gScore = {start: 0}						#g(x) do estado inicial é 0
		fScore = {start: util.manhattanDistance(start, problem.goal)}	#f(x) inicial é 0 + h(x), que é a heurística
		openSet.push(start, fScore[start])				#A borda é uma pilha de estados ordenada por f(x)

		candidate_state = None						#Estado candidato à ação do fantasma

		while not openSet.isEmpty():					#A*
			current = openSet.pop()
			closedSet.add(current)
			#Se chegarmos ao estado final, recriamos um caminho de estados até o início da busca, tirando o estado inicial
			if problem.isGoalState(current):			
				candidate_state = self.reconstruct_path(cameFrom, current)[-2]
				break

			#Expande o estado atual em até 4 vizinhos ( Norte, Sul, Leste e Oeste )
			for neighbor in problem.getSuccessors(current):
				
				isScared = False

				
				#Se há um fantasma no caminho, é verificado se este fantasma está com medo. Se estiver com medo, então a busca prossegue normalmente, caso o fantasma não esteja com medo, Pacman ignorará este caminho
				
				if neighbor[STATE] in ghosts:
					for index in xrange(1, state.getNumAgents()):
						if state.getGhostState(index).scaredTimer > 0 and state.getGhostPosition(index) == neighbor[STATE]:
							isScared = True

				if (neighbor[STATE] in ghosts and isScared) or neighbor[STATE] not in ghosts:

					if neighbor[STATE] in closedSet:	#Se este vizinho já foi visitado
						continue

					if not openSet.contains(neighbor[STATE]): #Se este vizinho não está na borda
					#calcula f(x) deste vizinho e o adiciona à borda
						fScore[neighbor[STATE]] = util.manhattanDistance(neighbor[STATE], problem.goal)
						openSet.push(neighbor[STATE], fScore[neighbor[STATE]])

					
					tentative_gScore = gScore[current] + neighbor[COST]
					#Verifica se ir para esse vizinho vale a pena
					if tentative_gScore >= gScore.get(neighbor[STATE], float("inf")):
						continue

					
					cameFrom[neighbor[STATE]] = current		#Guarda quem é o pai desse vizinho
					gScore[neighbor[STATE]] = tentative_gScore	#g(x) desse vizinho
					#f(x) = g(x) + h(x)
					fScore[neighbor[STATE]] = gScore[neighbor[STATE]] + util.manhattanDistance(neighbor[STATE], problem.goal)

		bestAction = None

		#Retorna a melhor ação legal
		for neighbor in problem.getSuccessors(start):
			if candidate_state == neighbor[STATE]:
				if neighbor[ACTION] in legalActions:
					bestAction = neighbor[ACTION]

		if bestAction == None:
			bestAction = random.choice(legalActions)

		return bestAction


	#Recria o caminho de nós expandidos até o nó atual
	def reconstruct_path(self, cameFrom, current):
		total_path = [current]
		while current in cameFrom.keys():
			current = cameFrom[current]
			total_path.append(current)
		return total_path """



"""class PacmanAgent(game.Agent):
    def getAction(self, state):
        problem = PositionSearchProblem(state)
        problem.startState = state.getPacmanPosition()
        start = problem.getStartState()

        comidaGrid = state.getFood()
        listaDeComidas = list(comidaGrid.asList())  # Lista com as coordenadas de todas as comidas.

        distancia, comida = min([(util.manhattanDistance(start, comida), comida) for comida in listaDeComidas])  # Obtem a comida mais proxima do Pacman utilizando a distancia de manhattan.
        problem.goal = comida

        pilha = util.PriorityQueue()
        pilha.push((start, []), 0)
        visitados = set([start]) #Conjunto dos percorridos

        while not pilha.isEmpty():
            noAtualTupla = pilha.pop()
            noAtual = noAtualTupla[0]
            caminho = noAtualTupla[1]

            if problem.isGoalState(noAtual):
                return caminho[0] #Retorna apenas a primeira acao da lista de acoes retornada pelo algoritmo A*.

            for coordAtual in problem.getSuccessors(noAtual):
                if not coordAtual[0] in state.getGhostPositions(): #Se tiver fantasmas no caminho a ser buscado, o ramo nao e inserido na fila.
                    coord, direcao, custo = coordAtual
                    tempPath = list(caminho)
                    if not coord in visitados:
                        visitados.add(coord) #Insere o noAtual no conjunto de visitados
                        tempPath.append(direcao)
                        custoTotal = problem.getCostOfActions(tempPath) + util.manhattanDistance(coord, problem.goal)
                        pilha.push((coord, tempPath), custoTotal)"""


class LeftTurnAgent(game.Agent):
  "An agent that turns left at every opportunity"
  
  def getAction(self, state):
    legal = state.getLegalPacmanActions()
    current = state.getPacmanState().configuration.direction
    if current == Directions.STOP: current = Directions.NORTH
    left = Directions.LEFT[current]
    if left in legal: return left
    if current in legal: return current
    if Directions.RIGHT[current] in legal: return Directions.RIGHT[current]
    if Directions.LEFT[left] in legal: return Directions.LEFT[left]
    return Directions.STOP

class GreedyAgent(Agent):
  def __init__(self, evalFn="scoreEvaluation"):
    self.evaluationFunction = util.lookup(evalFn, globals())
    assert self.evaluationFunction != None
        
  def getAction(self, state):
    # Generate candidate actions
    legal = state.getLegalPacmanActions()
    if Directions.STOP in legal: legal.remove(Directions.STOP)
      
    successors = [(state.generateSuccessor(0, action), action) for action in legal] 
    scored = [(self.evaluationFunction(state), action) for state, action in successors]
    bestScore = max(scored)[0]
    bestActions = [pair[1] for pair in scored if pair[0] == bestScore]
    return random.choice(bestActions)
  
def scoreEvaluation(state):
	print "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
	return state.getScore()  
