#include "stdafx.h"
#include "MyGame.h"

#pragma warning (disable: 4244)

#define SPEED_GUARD_0	80
#define SPEED_GUARD_1	80
#define SPEED_GUARD_2	80
#define SPEED_GUARD_3	80

int rotation = 1;


char* CMyGame::m_tileLayout[12] =
{
	"XXXXXXXXXXXXXXXXXXXX",
	"X                  X",
	"X XXX    XXX   XX  X",
	"X XXX    XXX   XXXXX",
	"X       XXX     XXXX",
	"XXXX    XX   XX   XX",
	"XXXX        XXX   XX",
	"X    XX    XXX   XXX",
	"X XX XXXX  XX    XXX",
	"X XX XXXX  XX    XXX",
	"                   X",
	"XXXXXXXXXXXXXXXXXXXX",
};

float Coords[][2] = {
{1190,90},
{1190,165},
{1115,165},
{1115,90},
{915,90},
{915,195},
{915,290},
{800,90},
{800,195},
{800,290},
{735,290},
{995,290},
{995,350},
{1115,350},
{1115,415},
{995,415},
{735,350},
{800,350},
{670,350},
{735,415},
{670,415},
{670,480},
{485,415},
{485,480},
{1050,480},
{925,480},
{540,90},
{540,220},
{360,90},
{360,220},
{360,290},
{485,290},
{96,90},
{96,290},
{290,290},
{290,350},
{290,415},
{290,480},
{485,350},
{96,480},
{96,670},
{290,670},
{670,670},
{870,670},
{870,550},
{1050,550},
{1050,670},
{1190,670},
{10,670}

};

int Connections[][2] = {
	
{0,1},
{1,2},
{2,0},
{1,3},
{0,3},
{2,3},
{3,4},
{4,5},
{5,6},
{4,7},
{4,8},
{7,8},
{7,5},
{5,8},
{5,9},
{8,9},
{6,9},
{9,10},
{10,17},
{9,17},
{10,16},
{16,9},
{17,16},
{16,19},
{16,18},
{18,19},
{18,20},
{19,20},
{20,16},
{20,21},
{20,22},
{22,23},
{22,21},
{23,21},
{23,20},
{22,38},
{38,31},
{22,36},
{36,35},
{35,34},
{34,30},
{31,30},
{36,38},
{22,35},
{30,29},
{29,31},
{29,27},
{27,26},
{35,38},
{29,28},
{28,26},
{29,26},
{27,28},
{28,32},
{32,33},
{33,34},
{36,37},
{37,39},
{39,40},
{40,41},
{37,41},
{41,42},
{42,21},
{42,43},
{43,46},
{46,47},
{43,44},
{44,45},
{43,45},
{46,44},
{35,31},
{38,34},
{46,45},
{44,24},
{45,25},
{24,15},
{15,14},
{15,13},
{15,12},
{14,13},
{12,13},
{12,11},
{11,6},
{45,24},
{25,24},
{14,12},
{6,8},
{26,7},
{30,27},
{40,48},
{25,44}
};


bool Intersection(CVector a, CVector b, CVector c, CVector d, float& k1, float& k2)
{
	CVector v1 = b - a;
	CVector v2 = d - c;
	CVector con = c - a;
	float det = v1.m_x * v2.m_y - v1.m_y * v2.m_x;
	if (det != 0)
	{
		k1 = (v2.m_y * con.m_x - v2.m_x * con.m_y) / det;
		k2 = (v1.m_y * con.m_x - v1.m_x * con.m_y) / det;
		return true;
	}
	else
		return false;
}

bool Intersection(CVector a, CVector b, CVector c, CVector d)
{
	float k1, k2;
	if (!Intersection(a, b, c, d, k1, k2))
		return false;
	return k1 >= 0 && k1 <= 1.f && k2 >= 0 && k2 <= 1.f;
}

/////////////////////////////////////////////////////
// Dijkstra algorithm implementation

bool PathFind(vector<NODE>& graph, int nStart, int nGoal, vector<int>& path)
{
	list<int> open;

	// mark all nodes in the graph as unvisited
	for (unsigned i = 0; i < graph.size(); i++)
		graph[i].open = false;

	// open the Start node
	graph[nStart].costSoFar = 0;
	graph[nStart].nConnection = -1;
	graph[nStart].open = true;
	open.push_back(nStart);

	while (open.size() > 0)
	{
		// Find the element with the smallest costSoFar
		// iMin is the iterator (pointer) to its position in the opn list
		list<int>::iterator iCurrent = min_element(open.begin(), open.end(), [graph](int i, int j) -> bool {
			return graph[i].costSoFar < graph[j].costSoFar;
			});
		int curNode = *iCurrent;
		float coastSoFar = graph[curNode].costSoFar;

		// If the end node found, then terminate
		if (curNode == nGoal)
			break;

		// Otherwise, visit all the connections
		for (CONNECTION conn : graph[curNode].conlist)
		{
			int endNode = conn.nEnd;
			float newCostSoFar = coastSoFar + conn.cost;

			// for open nodes, ignore if the current route worse then the route already found
			if (graph[endNode].open && graph[endNode].costSoFar <= newCostSoFar)
				continue;

			// Wow, we've found a better route!
			graph[endNode].costSoFar = newCostSoFar;
			graph[endNode].nConnection = curNode;

			// if unvisited yet, add to the open list
			if (!graph[endNode].open)
			{
				graph[endNode].open = true;
				open.push_back(endNode);
			}

			// in Dijkstra, this should never be a closed node
		}

		// We can now close the current graph...
		graph[curNode].closed = true;
		open.erase(iCurrent);
	}

	// Collect the path from the generated graph data
	if (open.size() == 0)
		return false;		// path not found!

	int i = nGoal;
	while (graph[i].nConnection >= 0)
	{
		path.push_back(i);
		i = graph[i].nConnection;
	}
	path.push_back(i);

	reverse(path.begin(), path.end());
	return true;
}

CMyGame::CMyGame(void) :
	m_player(1190, 90, 64, 64, 0)
{
	seen = NULL;
	m_player.LoadAnimation("Spider64.png", "walk", CSprite::Sheet(4, 2).Col(0).From(0).To(1));
	m_player.LoadAnimation("Spider64.png", "idle", CSprite::Sheet(4, 2).Col(2).From(0).To(1));
	m_player.SetAnimation("idle", 4);

	// create graph structure - nodes
	for (float* coord : Coords)
		m_graph.push_back(NODE{ CVector(coord[0], coord[1]) });

	// create graph structure - connections
	for (int* conn : Connections)
	{
		int ind1 = conn[0];
		int ind2 = conn[1];
		NODE& node1 = m_graph[ind1];
		NODE& node2 = m_graph[ind2];
		float dist = Distance(node1.pos, node2.pos);

		node1.conlist.push_back(CONNECTION{ ind2, dist });
		node2.conlist.push_back(CONNECTION{ ind1, dist });
	}


}

CMyGame::~CMyGame(void)
{
}

/////////////////////////////////////////////////////
// Per-Frame Callback Funtions (must be implemented!)

void CMyGame::OnUpdate()
{
	Uint32 t = GetTime();

	// player: follow the waypoints
	if (!m_waypoints.empty())
	{
		// If player not moving, start moving to the first waypoint
		if (m_player.GetSpeed() < 1)
		{
			m_player.SetSpeed(500);
			m_player.SetAnimation("walk");
			m_player.SetDirection(m_waypoints.front() - m_player.GetPosition());
			m_player.SetRotation(m_player.GetDirection() - 90);
		}

		// Passed the waypoint?
		CVector v = m_waypoints.front() - m_player.GetPosition();
		if (Dot(m_player.GetVelocity(), v) < 0)
		{
			// Stop movement
			m_waypoints.pop_front();
			if (m_waypoints.empty())
				m_player.SetAnimation("idle");
			m_player.SetVelocity(0, 0);
			m_player.SetRotation(0);
		}
	}
	


	if (m_guards[0]->GetPosition().m_x < 96)
	{
		m_guards[0]->SetAnimation("walkR");
		m_guards[0]->SetVelocity(CVector(SPEED_GUARD_0, 0));
		
	}

	if (m_guards[0]->GetPosition().m_x > 480)
	{
		m_guards[0]->SetAnimation("walkL");
		m_guards[0]->SetVelocity(CVector(-SPEED_GUARD_0, 0));

	}

	

	

	if (m_guards[1]->GetPosition().m_y > 670)
	{
		m_guards[1]->SetAnimation("walkD");
		m_guards[1]->SetVelocity(CVector(0, -SPEED_GUARD_1));
	}
	if (m_guards[1]->GetPosition().m_y < 290)
	{
		m_guards[1]->SetAnimation("walkU");
		m_guards[1]->SetVelocity(CVector(0, SPEED_GUARD_1));
	}

	if (m_guards[2]->GetPosition().m_x > 1190)
	{
		m_guards[2]->SetAnimation("walkL");
		m_guards[2]->SetVelocity(CVector(-SPEED_GUARD_2, 0));
	}
	if(m_guards[2]->GetPosition().m_x < 90)
	{
		m_guards[2]->SetAnimation("walkR");
		m_guards[2]->SetVelocity(CVector(SPEED_GUARD_2, 0));
	}


	if (m_guards[3]->GetPosition().m_x > 620)
	{
		m_guards[3]->SetAnimation("walkL");
		m_guards[3]->SetVelocity(CVector(-SPEED_GUARD_3, 0));
	}
	if (m_guards[3]->GetPosition().m_x < 290)
	{
		m_guards[3]->SetAnimation("walkR");
		m_guards[3]->SetVelocity(CVector(SPEED_GUARD_3, 0));
	}

	for (CSprite* pGuard : m_guards)
		pGuard->Update(GetTime());

	for (CSprite* pGuard : m_guards)
	{
		// by default, we assume each guard can become a killer
		m_pKiller = pGuard;

		// browse through all tiles - if line of sight test shows any tile to obscure the player, then we have no killer after all
		for (CSprite* pTile : m_tiles)
		{
			CVector e(pTile->GetLeft(), pTile->GetBottom());
			CVector f(pTile->GetRight(), pTile->GetTop());
			// Check intersection of the "Guard - Player" sight line with both diagonals of the tile.
			// If there is intersection - there is no killer - so, m_pKiller = NULL;
			if (Intersection(pGuard->GetPosition(), m_player.GetPosition(), CVector(pTile->GetLeft(), pTile->GetTop()), CVector(pTile->GetRight(), pTile->GetBottom())))
				m_pKiller = NULL;
			if (Intersection(pGuard->GetPosition(), m_player.GetPosition(), e, f))
				m_pKiller = NULL;

			if (m_pKiller == NULL)
				break;	// small optimisation, if line of sight test already failed, no point to look further
		}

		// if the player is in plain sight of the guard...
		if (m_pKiller)
		{
			// Additional test - only killing if the player within 60 degrees from the guard's front (they have no eyes in the back of their head)
			CVector v = m_player.GetPosition() - pGuard->GetPosition();
			if (Dot(Normalise(v), Normalise(pGuard->GetVelocity())) <= (sqrt(2)/1.5))
				m_pKiller = NULL;
		}

		// if still the killer found - the game is over and look no more!
		if (m_pKiller)
		{
			GameOver();
			return;
		}

		// WINNING TEST
		if (m_player.GetLeft() < 11)
			GameOver();

		
	}

	
	m_player.Update(t);
}

void CMyGame::OnDraw(CGraphics* g)
{
	for (NODE n : m_graph)
		for (CONNECTION c : n.conlist)
			g->DrawLine(n.pos, m_graph[c.nEnd].pos, CColor::Black());
	m_nodes.for_each(&CSprite::Draw, g);
	m_tiles.for_each(&CSprite::Draw, g);
	m_player.Draw(g);

	for (CSprite* pGuard : m_guards)
		pGuard->Draw(g);

	if (m_pKiller)
	{
		g->DrawLine(m_pKiller->GetPosition(), m_player.GetPosition(), 4, CColor::Red());
		*g << font(48) << color(CColor::Red()) << vcenter << center << "YOU GOT CAUGHT!" << endl;
	}
	else if (IsGameOver())
		*g << font(48) << color(CColor::DarkBlue()) << vcenter << center << "YOU ESCAPED!" << endl;
}

/////////////////////////////////////////////////////
// Game Life Cycle

// one time initialisation
void CMyGame::OnInitialize()
{
	seen = NULL;
	// Create Tiles
	for (int y = 0; y < 12; y++)
		for (int x = 0; x < 20; x++)
		{
			if (m_tileLayout[y][x] == ' ')
				continue;

			int nTile = 5;
			if (y > 0 && m_tileLayout[y - 1][x] == ' ') nTile -= 3;
			if (y < 11 && m_tileLayout[y + 1][x] == ' ') nTile += 3;
			if (x > 0 && m_tileLayout[y][x - 1] == ' ') nTile--;
			if (x < 20 && m_tileLayout[y][x + 1] == ' ') nTile++;
			if (nTile == 5 && x > 0 && y > 0 && m_tileLayout[y - 1][x - 1] == ' ') nTile = 14;
			if (nTile == 5 && x < 20 && y > 0 && m_tileLayout[y - 1][x + 1] == ' ') nTile = 13;
			if (nTile == 5 && x > 0 && y < 11 && m_tileLayout[y + 1][x - 1] == ' ') nTile = 11;
			if (nTile == 5 && x < 20 && y < 11 && m_tileLayout[y + 1][x + 1] == ' ') nTile = 10;
			
			nTile--;
			m_tiles.push_back(new CSprite(x * 64.f + 32.f, y * 64.f + 32.f, new CGraphics("tiles.png", 3, 5, nTile % 3, nTile / 3), 0));
		}


	// Create Nodes
	int i = 0;
	for (NODE n : m_graph)
	{
		stringstream s;
		s << i++;
		m_nodes.push_back(new CSpriteOval(n.pos, 12, CColor::White(), CColor::Black(), 0));
		m_nodes.push_back(new CSpriteText(n.pos, "arial.ttf", 14, s.str(), CColor::Black(), 0));
	}


	for (int i = 0; i < 4; i++)
	{
		CSprite* pGuard = new CSprite(0, 0, "guard.png", 0);
		pGuard->LoadAnimation("guard.png", "walkR", CSprite::Sheet(13, 21).Row(9).From(0).To(8));
		pGuard->LoadAnimation("guard.png", "walkD", CSprite::Sheet(13, 21).Row(10).From(0).To(8));
		pGuard->LoadAnimation("guard.png", "walkL", CSprite::Sheet(13, 21).Row(11).From(0).To(8));
		pGuard->LoadAnimation("guard.png", "walkU", CSprite::Sheet(13, 21).Row(12).From(0).To(8));
		m_guards.push_back(pGuard);


	}
}

// called when a new game is requested (e.g. when F2 pressed)
// use this function to prepare a menu or a welcome screen
void CMyGame::OnDisplayMenu()
{
	StartGame();	// exits the menu mode and starts the game mode
}

// called when a new game is started
// as a second phase after a menu or a welcome screen
void CMyGame::OnStartGame()
{
	m_guards[0]->SetPosition(360, 290);
	m_guards[0]->SetAnimation("walkL");
	m_guards[0]->SetVelocity(CVector(-SPEED_GUARD_0, 0));

	m_guards[1]->SetPosition(995, 290);
	m_guards[1]->SetAnimation("walkU");
	m_guards[1]->SetVelocity(CVector(0, SPEED_GUARD_1));

	m_guards[2]->SetPosition(96, 670);
	m_guards[2]->SetAnimation("walkR");
	m_guards[2]->SetVelocity(CVector(SPEED_GUARD_2, 0));

	m_guards[3]->SetPosition(420, 415);
	m_guards[3]->SetAnimation("walkR");
	m_guards[3]->SetVelocity(CVector(SPEED_GUARD_3, 0));

	m_pKiller = NULL;




}

// called when a new level started - first call for nLevel = 1
void CMyGame::OnStartLevel(Sint16 nLevel)
{
}

// called when the game is over
void CMyGame::OnGameOver()
{
}

// one time termination code
void CMyGame::OnTerminate()
{
}

/////////////////////////////////////////////////////
// Keyboard Event Handlers

void CMyGame::OnKeyDown(SDLKey sym, SDLMod mod, Uint16 unicode)
{
	if (sym == SDLK_F4 && (mod & (KMOD_LALT | KMOD_RALT)))
		StopGame();
	if (sym == SDLK_SPACE)
		PauseGame();
	if (sym == SDLK_F2)
		NewGame();
}

void CMyGame::OnKeyUp(SDLKey sym, SDLMod mod, Uint16 unicode)
{
}


/////////////////////////////////////////////////////
// Mouse Events Handlers

void CMyGame::OnMouseMove(Uint16 x,Uint16 y,Sint16 relx,Sint16 rely,bool bLeft,bool bRight,bool bMiddle)
{
}

void CMyGame::OnLButtonDown(Uint16 x, Uint16 y)
{
	CVector v(x, y);	// destination

	// check if the move is legal
	if (m_tileLayout[y / 64][x / 64] != ' ')
		return;	// cannot go into a wall!

	// find the first node: the closest to the player
	vector<NODE>::iterator iFirst =
		min_element(m_graph.begin(), m_graph.end(), [this](NODE& n1, NODE& n2) -> bool {
		return Distance(n1.pos, m_player.GetPos()) < Distance(n2.pos, m_player.GetPos());
			});

	// find the last node: the closest to the destination
	vector<NODE>::iterator iLast =
		min_element(m_graph.begin(), m_graph.end(), [v](NODE& n1, NODE& n2) -> bool {
		return Distance(n1.pos, v) < Distance(n2.pos, v);
			});

	int nFirst = iFirst - m_graph.begin();
	int nLast = iLast - m_graph.begin();


	// remove the current way points and reset the player
	if (!m_waypoints.empty())
	{
		m_waypoints.clear();
		m_player.SetVelocity(0, 0);
	}

	// call the path finding algorithm to complete the waypoints
	vector<int> path;
	if (PathFind(m_graph, nFirst, nLast, path))
	{
		for (int i : path)
			m_waypoints.push_back(m_graph[i].pos);
		m_waypoints.push_back(v);
	}
}

void CMyGame::OnLButtonUp(Uint16 x,Uint16 y)
{
}

void CMyGame::OnRButtonDown(Uint16 x,Uint16 y)
{
}

void CMyGame::OnRButtonUp(Uint16 x,Uint16 y)
{
}

void CMyGame::OnMButtonDown(Uint16 x,Uint16 y)
{
}

void CMyGame::OnMButtonUp(Uint16 x,Uint16 y)
{
}
