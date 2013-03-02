// Copyright (c) 2011, Christoph Schunk
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the author nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

package astar

import (
	"container/heap"
)

type HeuristicCostEstimateFunc func(node, goal Node) float32

type none struct {
}

type Node interface {
	NumNeighbours() int
	Neighbour(i int) Node
	Cost(i int) float32
}

type nodeElement struct {
	F        float32
	G        float32
	H        float32
	Node     Node
	cameFrom *nodeElement
}

type nodeElementPool []nodeElement
type nodeElementHeap []*nodeElement

type AStar struct {
	costEstimateFunc HeuristicCostEstimateFunc
	elementPool      nodeElementPool
	openList         nodeElementHeap
	closedList       map[Node]none
	path             []Node
}

func (p *nodeElementPool) NewElement(f, g, h float32, node Node, cameFrom *nodeElement) *nodeElement {
	(*p) = append(*p, nodeElement{f, g, h, node, cameFrom})
	return &(*p)[len(*p)-1]
}

func (ol *nodeElementHeap) Len() int {
	return len(*ol)
}

func (ol *nodeElementHeap) Less(i, j int) bool {
	return (*ol)[i].F < (*ol)[j].F
}

func (ol *nodeElementHeap) Swap(i, j int) {
	(*ol)[i], (*ol)[j] = (*ol)[j], (*ol)[i]
}

func (ol *nodeElementHeap) Push(e interface{}) {
	(*ol) = append(*ol, e.(*nodeElement))
}

func (ol *nodeElementHeap) Pop() interface{} {
	e := (*ol)[len(*ol)-1]
	(*ol) = (*ol)[:len(*ol)-1]
	return e
}

func New(costEstimateFunc HeuristicCostEstimateFunc) *AStar {
	return &AStar{
		costEstimateFunc,
		make(nodeElementPool, 0, 1024),
		make(nodeElementHeap, 0, 256),
		make(map[Node]none),
		make([]Node, 0, 128)}
}

// Returns the path including the start and the end node if Find() was true.
// Note that the returned slice of path nodes may be invalidated after the next call to Find().
func (as *AStar) Path() []Node {
	return as.path
}

func (as *AStar) reset() {
	as.elementPool = as.elementPool[:0]
	as.openList = as.openList[:0]
	as.closedList = make(map[Node]none)
	as.path = as.path[:0]
}

// Finds the shortest path between two nodes in a graph.
func (as *AStar) Find(start, goal Node) bool {
	as.reset()
	g := float32(0)
	h := as.costEstimateFunc(start, goal)
	heap.Push(&as.openList, as.elementPool.NewElement(g+h, g, h, start, nil))
	for len(as.openList) > 0 {
		currentNodeElement := heap.Pop(&as.openList).(*nodeElement)
		if currentNodeElement.Node == goal {
			as.reconstructPath(currentNodeElement)
			return true
		}
		as.expandNode(currentNodeElement, goal)
		as.closedList[currentNodeElement.Node] = none{}
	}
	return false
}

func (as *AStar) expandNode(currentNodeElement *nodeElement, goal Node) {
	numNeigbours := currentNodeElement.Node.NumNeighbours()
	for i := 0; i < numNeigbours; i++ {
		successorNode := currentNodeElement.Node.Neighbour(i)
		if _, ok := as.closedList[successorNode]; ok {
			continue
		}
		t := currentNodeElement.G + currentNodeElement.Node.Cost(i)
		if index, ok := as.inOpenList(successorNode); ok {
			g := as.openList[index].G
			if t < g {
				g, h := t, as.costEstimateFunc(successorNode, goal)
				elem := heap.Remove(&as.openList, index).(*nodeElement)
				elem.G, elem.H, elem.F = g, h, g+h
				elem.cameFrom = currentNodeElement
				heap.Push(&as.openList, elem)
			}
		} else {
			g, h := t, as.costEstimateFunc(successorNode, goal)
			heap.Push(&as.openList, as.elementPool.NewElement(g+h, g, h, successorNode, currentNodeElement))
		}
	}
}

func (as *AStar) reconstructPath(currentNodeElement *nodeElement) {
	nextNode := currentNodeElement
	for nextNode != nil {
		as.path = append(as.path, nextNode.Node)
		nextNode = nextNode.cameFrom
	}
}

func (as *AStar) inOpenList(n Node) (index int, ok bool) {
	for i, node := range as.openList {
		if n == node.Node {
			return i, true
		}
	}
	return -1, false
}
