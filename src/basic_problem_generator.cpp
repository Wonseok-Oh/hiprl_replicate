#include <iostream>
#include <fstream>
#include <map>
using namespace std;

class cell {

	string m_name;

public:
	cell(){
		m_name = string("err");
	}
	cell(string name){
		m_name = name;
	}
	~cell(){};

	bool setName(string name){
		m_name = name;
		return true;
	}

	bool getName(string& name) const {
		name = m_name;
		return true;
	}
	map<string, cell*> connections;

};

bool generate_location_list(int size, string& seq){
	seq = string("");
	for (int i = 0; i < size; i++){
		for (int j = 0; j < size; j++){
			seq = seq + string(" f") + to_string(i) + string("-") + to_string(j) + string("f");
		}
	}
	return true;
}

bool generate_conn_predicates(int size, string& seq){
	cell graph[size*size];
	string conn_predicates("");
	// construct graph
	for (int y = 0; y < size; y++){
		for (int x = 0; x < size; x++){
			string name = string("f") + to_string(y) + string("-") + to_string(x) + string("f");
			graph[y*size + x].setName(name);

			// check adjacent cells and connect them by pointer
			// if right cell exists
			if (x + 1 < size){
				graph[y*size + x].connections.insert(make_pair("right", &graph[y*size + x + 1]));
			}

			// if left cell exists
			if (x > 0){
				graph[y*size + x].connections.insert(make_pair("left", &graph[y*size + x - 1]));
			}

			// if up cell exists
			if (y > 0){
				graph[y*size + x].connections.insert(make_pair("up", &graph[(y-1)*size + x]));
			}

			// if down cell exists
			if (y + 1 < size){
				graph[y*size + x].connections.insert(make_pair("down", &graph[(y+1)*size + x]));
			}
		}
	}

	// print string & return
	for (int y = 0; y < size; y++){
		for (int x = 0; x < size; x++){
			cell current = graph[y*size+x];
			string name, itr_name;
			bool gotName = current.getName(name);
			map<string, cell*>::iterator itr = graph[y*size + x].connections.begin();
			for (; itr != graph[y*size + x].connections.end(); itr++){
				gotName = itr->second->getName(itr_name);
				seq = seq + string("\n        (conn ") + name + string(" ") + itr_name + string(" ") + itr->first + string(")");
			}
		}
	}

	return true;
}

int main(int argc, char** argv){
	int size = 20;
	string list_of_locations, connections;
	bool got_location_list = generate_location_list(size, list_of_locations);
	bool got_connections = generate_conn_predicates(size, connections);
	string filePath = string("/home/morin/catkin_ws/src/hiprl_replicate/pddl/hiprl_problem0_multi.pddl");
	ofstream writeFile(filePath.data());
	if (writeFile.is_open()){
		writeFile << "(define (problem hiprl_problem_mini_multi)\n";
		writeFile << "    (:domain hiprl_mini_multi)\n";
		writeFile << "    (:objects\n";
		writeFile << (string("       ") + list_of_locations + string(" - location\n")).c_str();
		writeFile << "        robot0 robot1 robot2 - agent\n";
		writeFile << "        box0 - receptacle\n";
		writeFile << "        ball0 - obj\n";
		writeFile << "        ball_type - otype\n";
		writeFile << "        box_type - rtype\n";
		writeFile << "        down left right up - direction\n";
		writeFile << "    )\n";
		writeFile << "    (:init\n";
		writeFile << "        (at_location robot0 f0-0f)\n";
		writeFile << "        (at_location robot1 f0-0f)\n";
		writeFile << "        (at_location robot2 f0-0f)\n";

		writeFile << "        (handsfree robot0)\n";
		writeFile << "        (handsfree robot1)\n";
		writeFile << "        (handsfree robot2)\n";

		writeFile << "        (object_type ball0 ball_type)\n";
		writeFile << connections;
		writeFile << "    )\n";
		writeFile << "\n";
		writeFile << "    (:goal\n";
		writeFile << "        (object_at_location ball0 f0-0f)\n";
		writeFile << "    )\n";
		writeFile << ")";
		writeFile.close();
	}
	return 0;
}
