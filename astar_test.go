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
	"math"
	"testing"
)

var (
	costs = []float32{1, 1.141, 1.0, 1.141, 1.0, 1.141, 1.0, 1.141}
	dirx  = []int{0, 1, 1, 1, 0, -1, -1, -1}
	diry  = []int{-1, -1, 0, 1, 1, 1, 0, -1}
)

type MNode struct {
	m    *Map
	x, y int
	cost float32
}

type Map struct {
	nodes []MNode
	w, h  int
}

func NewMap(w, h int) *Map {
	m := &Map{make([]MNode, w*h), w, h}
	for i, _ := range m.nodes {
		n := &m.nodes[i]
		n.m = m
		n.cost = 1.0
		n.x = i % w
		n.y = i / w
	}
	return m
}

func (m *Map) Node(x, y int) *MNode {
	if x < 0 {
		x += m.w
	} else if x >= m.w {
		x -= m.w
	}
	if y < 0 {
		y += m.h
	} else if y >= m.h {
		y -= m.h
	}
	return &m.nodes[x+y*m.w]
}

func (n *MNode) NumNeighbours() int {
	return 8
}

func (n *MNode) Cost(i int) float32 {
	return n.m.Node(n.x+dirx[i], n.y+diry[i]).cost * costs[i]
}

func (n *MNode) Neighbour(i int) Node {
	return n.m.Node(n.x+dirx[i], n.y+diry[i])
}

func Heuristic(node, goal Node) float32 {
	s := node.(*MNode)
	g := goal.(*MNode)
	dx := (s.x - g.x)
	dy := (s.y - g.y)
	return float32(math.Sqrt(float64(dx*dx + dy*dy)))
}

type pos struct {
	x, y int
}

type pathTest struct {
	start, end pos
	path       []pos
}

var pathTests = []pathTest{
	{pos{0, 0}, pos{0, 0}, []pos{{0, 0}}},
	{pos{0, 0}, pos{1, 1}, []pos{{1, 1}, {0, 0}}},
	{pos{1, 1}, pos{5, 1}, []pos{{5, 1}, {4, 1}, {3, 1}, {2, 1}, {1, 1}}},
}

func TestAStar(t *testing.T) {
	as := New(Heuristic)
	m := NewMap(32, 32)
	for i, _ := range pathTests {
		tst := &pathTests[i]
		if !as.Find(m.Node(tst.start.x, tst.start.y), m.Node(tst.end.x, tst.end.y)) {
			t.Fatal("path not found")
		}
		for i, n := range as.Path() {
			mn := n.(*MNode)
			if mn.x != tst.path[i].x || mn.y != tst.path[i].y {
				t.Fatalf("wrong path at %d: node: %v path node: %v", i, mn, tst.path[i])
			}
		}
	}
}
