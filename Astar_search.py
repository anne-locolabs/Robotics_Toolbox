#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr 29 15:35:50 2022

@author: annemac
"""

import numpy as np
import csv

"""
Nodes class definition
"""
class Nodes:
    def __init__(self, ID, past_cost, heuristic_ctg, estimated_tc, parent):
        self.ID = ID
        self.past_cost = past_cost
        self.heuristic_ctg = heuristic_ctg
        self.estimated_tc = estimated_tc
        self.parent = parent
        
    def __repr__(self):
        return '{' + str(self.ID) + ', ' + str(self.past_cost) + ', ' + str(self.heuristic_ctg) \
            + ', ' +str(self.estimated_tc) + ', ' + str(self.parent) +'}' 
            
"""
cost matrix generation function

The matrix is constructed from the eges.csv file. It encodes the set of edges,
where a positive value corresponds to the cost of moving from node1 to node2. A
negative value indicates that no edge exists.
"""
def cost(edges,nodes_count):
    cost_mat=-1*np.ones((nodes_count,nodes_count))
    for row in edges:
        cost_mat[int(row[0])-1, int(row[1])-1]= float(row[2])
        cost_mat[int(row[1])-1, int(row[0])-1]= float(row[2])
    return cost_mat    


"""
function astar

The function performs a A* search on a maze

     :param nodes_file: nodes csv file (with path) - CopeliaSim compatible format
     :param edges_file: edges csv file (with path) - CopeliaSim compatible 
     :param start: ID of node used as starting point - expressed as an integer
     :param end: ID of node used as goal - expressed as an 
     :return path expressed as list of nodes (starting from start)
     :output path csv file - CopeliaSim compatible format
     
"""
def astar(nodes_file, edges_file, start, end):
   
    # read the csv files
    csv_edges=open(edges_file)
    edges=csv.reader(csv_edges)
    csv_nodes=open(nodes_file)
    nodes=csv.reader(csv_nodes)
    
    # creates nodes list
    nodes_list=[] 
    for row in nodes:
      node=Nodes(int(row[0]), 1E10, float(row[3]),0, 0)
      nodes_list.append(node)
    
    # calculates the number of nodes
    nodes_count = len(nodes_list)
    
    # creates cost matrix
    cost_mat=cost(edges, nodes_count)
    
    #Initialization of the open and closed nodes lists
    opened=[]
    for node in nodes_list:
        if node.ID==start:
            node.past_cost=0
            node.estimated_tc=node.past_cost + node.heuristic_ctg
            opened.append(node)
    closed=[]
    
    # Loop until the open list is empty
    while len(opened) > 0:
        
        # Sort the list so the first node in opened is the one that minimizes
        # the total estimated cost
        opened.sort(key=lambda x: x.estimated_tc)
    
        # The node with the lowest cost is removed from open, and called current.
        # The current node is added to closed.        
        current = opened.pop(0)
        closed.append(current)
        
        # Finish the search if current is the end and generates path
        if current.ID==end:
            path=[]
            while current.ID != start:
                path.append(str(current.ID))
                for node in nodes_list:
                    if node.ID == current.parent:
                        current=node
            path.append(str(current.ID))            
            Path=path[::-1]
            
            with open("path.csv","w") as path_csv:
                writer=csv.writer(path_csv)
                writer.writerow(Path)
            
            return  Path          
        
        # Find the current node's neighbors        
        nbr=[]
        for i in range(nodes_count):
            if cost_mat[current.ID-1,i]>=0:
                nbr.append(i+1)
        
        # loop over neighbors
        for n in nbr:
            
           # Identify n node in nodes_list
           for node in nodes_list:
               if node.ID == n:
                   n_list=node
            
           # Check if the neighbor is in the closed list
           if(n_list in closed):
                continue
            
           #calculate tentative cost 
           tent_cost=current.past_cost + cost_mat[current.ID-1, n_list.ID -1]
           
           # Tests node for least cost
           if tent_cost < n_list.past_cost:
               n_list.past_cost=tent_cost
               n_list.estimated_tc=n_list.past_cost + n_list.heuristic_ctg
               n_list.parent=current.ID
               opened.append(n_list)
    
    # Close the csv files           
    edges_file.close() 
    nodes_file.close()  
      
"""
APPLICATION TO MAZE
"""

edges_file="C:/Users/a lab in the Air/Desktop/Modern Robotics 4/mr4_assignment/results/edges.csv" 
nodes_file="C:/Users/a lab in the Air/Desktop/Modern Robotics 4/mr4_assignment/results/nodes.csv"

start=1
end=12

print(astar(nodes_file, edges_file, start, end))        
