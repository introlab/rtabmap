/**********************************************************************
 *
 * This source code is part of the Tree-based Network Optimizer (TORO)
 *
 * TORO Copyright (c) 2007 Giorgio Grisetti, Cyrill Stachniss,
 *                         Slawomir Grzonka and Wolfram Burgard
 *
 * TORO is licences under the Common Creative License,
 * Attribution-NonCommercial-ShareAlike 3.0
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *  
 *   - Noncommercial. You may not use this work for commercial purposes.
 *  
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 *
 * TORO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 **********************************************************************/

/** \file posegraph.hxx
 * 
 * \brief The implementation of the template class for the node
 * parameters. 
 **/



/*********************** IMPLEMENTATION PART ***********************/
template <typename Ops>
typename TreePoseGraph<Ops>::Vertex* TreePoseGraph<Ops>::vertex(int id){
  typename VertexMap::iterator it=vertices.find(id);
  if (it==vertices.end())
    return 0;
  return it->second;
}

template <typename Ops>
const typename TreePoseGraph<Ops>::Vertex * TreePoseGraph<Ops>::vertex (int id) const{
  typename VertexMap::const_iterator it=vertices.find(id);
  if (it==edges.end())
    return 0;
  return it->second;
}

template <class Ops>
typename TreePoseGraph<Ops>::Edge* TreePoseGraph<Ops>::edge(int id1, int id2){
  Vertex* v1=vertex(id1);
  if (!v1)
    return 0;
  typename EdgeList::iterator it=v1->edges.begin();
  while(it!=v1->edges.end()){
    if ((*it)->v1->id==id1 && (*it)->v2->id==id2)
      return *it;
    it++;
  }
  return 0;
}

template <class Ops>
const typename TreePoseGraph<Ops>::Edge * TreePoseGraph<Ops>::edge(int id1, int id2) const{
  const Vertex* v1=vertex(id1);
  if (!v1)
    return false;
  typename EdgeList::const_iterator it=v1->edges.begin();
  while(it!=v1->edges.end()){
    if ((*it)->v1->id==id1 && (*it)->v2->id==id2)
      return *it;
    it++;
  }
  return 0;
}

template <class Ops>
void TreePoseGraph<Ops>::revertEdge(typename TreePoseGraph<Ops>::Edge * e){
  revertEdgeInfo(e);
  Vertex* ap=e->v2;
  e->v2=e->v1;
  e->v1=ap;
}

template <class Ops>
typename TreePoseGraph<Ops>::Vertex* TreePoseGraph<Ops>::addVertex(int id, const typename TreePoseGraph<Ops>::Pose& pose){
  Vertex* v=vertex(id);
  if (v)
    return 0;
  v=new Vertex;
  v->id=id;
  v->pose=pose;
  v->parent=0;
  v->mark=false;
  vertices.insert(std::make_pair(id,v));
  return v;
}

template <class Ops>
typename TreePoseGraph<Ops>::Vertex* TreePoseGraph<Ops>::removeVertex (int id){
  typename VertexMap::iterator it=vertices.find(id);
  if (it==vertices.end())
    return 0;
  Vertex* v=it->second;

  if (v==0)
    return 0;

  typename TreePoseGraph<Ops>::EdgeList el=v->edges;
  for(typename EdgeList::iterator it=el.begin(); it!=el.end(); it++){
    removeEdge(*it);
  }
  delete v;
  vertices.erase(it);
  return v;
}

template <class Ops>
typename TreePoseGraph<Ops>::Edge* TreePoseGraph<Ops>::addEdge(typename TreePoseGraph<Ops>::Vertex* v1, typename TreePoseGraph<Ops>::Vertex* v2,
								 const typename TreePoseGraph<Ops>::Transformation& t, const typename TreePoseGraph<Ops>::Information& i){
  if (v1==v2)
    return 0;
  Edge* e=edge(v1->id, v2->id);
  if (e)
    return 0;

  e=new Edge;
  e->mark=false;
  e->v1=v1;
  e->v2=v2;
  e->top=0;
  e->transformation=t;
  e->informationMatrix=i;
  v1->edges.push_back(e);
  v2->edges.push_back(e);
  edges.insert(std::make_pair(e,e));
  return e;
}

template <class Ops>
typename TreePoseGraph<Ops>::Edge* TreePoseGraph<Ops>::addIncrementalEdge(int id1, int id2,
							       const typename TreePoseGraph<Ops>::Transformation& t, const typename TreePoseGraph<Ops>::Information& i){
  EVComparator<Edge*> comp;
  comp.mode=edgeCompareMode;
  if (! sortedEdges)
    sortedEdges=new EdgeSet(comp);


  typename VertexMap::iterator it1=vertices.find(id1);
  typename VertexMap::iterator it2=vertices.find(id2);
  Vertex* v1, *v2, *addedVertex=0;
  if (it1==vertices.end() && it2==vertices.end()){
    return 0;
  }

  if (it1==vertices.end()){
    typename TreePoseGraph<Ops>::Pose p;
    v1=addedVertex=addVertex(id1,p);
  } else {
    v1=it1->second;
  }
  if (it2==vertices.end()){
    typename TreePoseGraph<Ops>::Pose p;
    v2=addedVertex=addVertex(id2,p);
  } else {
    v2=it2->second;
  }

  if (v1->id==v2->id){
    assert(0);
  }

  Edge* e=addEdge(v1,v2,t,i);
  if (!e){
    return 0;
  }
  if (v1->id>v2->id)
    revertEdge(e);

  if (addedVertex){
    Vertex* otherVertex= (addedVertex==v1)? v2:v1;
    addedVertex->parent=otherVertex;
    addedVertex->parentEdge=e;
    addedVertex->level=otherVertex->level+1;
    otherVertex->children.push_back(e);
  }

  fillEdgeInfo(e);
  sortedEdges->insert(e);

  if (addedVertex){
    initializeFromParentEdge(addedVertex);
  }
  return e;
}


template <class Ops>
typename TreePoseGraph<Ops>::Edge* TreePoseGraph<Ops>::removeEdge(typename TreePoseGraph<Ops>::Edge* e){
  {
    typename EdgeMap::iterator it=edges.find(e);
    if (it==edges.end()){
      return 0;
    }
    edges.erase(it);
  }


  Vertex* v1=e->v1;
  Vertex* v2=e->v2;

  {
    typename EdgeList::iterator it=v1->edges.begin();
    while(it!=v1->edges.end()){
      if (*it==e){
	v1->edges.erase(it);
	break;
      }
      it++;
    }
  }

  {
    typename EdgeList::iterator it=v2->edges.begin();
    while(it!=v2->edges.end()){
      if ((*it)==e){
	delete *it;
	v2->edges.erase(it);
	break;
      }
      it++;
    }
  }

  return e;
}

template <class Ops>
template <class Action>
void TreePoseGraph<Ops>::treeBreadthVisit(Action& act){
  typedef std::deque<Vertex*> VertexDeque;
  static VertexDeque q;
  q.push_back(root);
  while (!q.empty()){
    Vertex* current=q.front();
    act.perform(current);
    q.pop_front();
    typename EdgeList::iterator it=current->children.begin();
    while(it!=current->children.end()){
      typename TreePoseGraph::Edge* e=(*it);
      q.push_back(e->v2);
      if(e->v2==current){
	std::cerr << "error in the link direction v=" << current->id << std::endl;
	std::cerr << " v1=" << e->v1->id << " v2=" << e->v2->id <<  std::endl;
	assert(0);
      }
      it++;
    }
  }
  q.clear();
}

template <class Ops>
template <class Action>
void TreePoseGraph<Ops>::treeDepthVisit(Action& act, Vertex* v){
  act.perform(v);
  typename EdgeList::iterator it=v->children.begin();
  while(it!=v->children.end()){
    treeDepthVisit(act, (*it)->v2);
    it++;
  }
}

template <class Ops>
bool TreePoseGraph<Ops>::buildMST(int id){
  typedef std::deque<Vertex*> VertexDeque;
  typename VertexMap::iterator it=vertices.begin();
  while (it!=vertices.end()){
    it->second->parent=0;
    it->second->parentEdge=0;
    it->second->children.clear();
    it++;
  }
  Vertex* v=vertex(id);
  if (!v)
    return false;
  root=v;
  root->level=0;
  VertexDeque q;
  q.push_back(v);
  //std::cerr << "v=" << v->id << std::endl;
  while (!q.empty()){
    v=q.front();
    typename EdgeList::iterator it=v->edges.begin();
    while (it!=v->edges.end()){
      Edge* e=(*it);
      bool invertedEdge=false;
      Vertex* other=e->v2;
      if (other==v){
	other=e->v1;
	invertedEdge=true;
      }
      if (other!=root && other->parent==0){
	if (invertedEdge){
	  revertEdge(e);
	}
	//std::cerr << "INSERT v=" << v->id<< " " << "e=(" << e->v1->id << "," << e->v2->id << ")" << std::endl;
	other->parent=v;
	other->parentEdge=e;
	other->level=v->level+1;
	q.push_back(other);
	v->children.push_back(e);
	//std::cerr << "v=" << other->id << std::endl;
      }
      it++;
    }
    q.pop_front();
  }
  fillEdgesInfo();
  return true;
}

/** \brief A class (struct) to dermine the level of a vertex in the tree **/
template <class TPG>
struct LevelAssigner{
  /** Dermines the level of the vertex v in the tree **/
  void perform(typename TPG::Vertex* v){
    if (v->parent)
      v->level=v->parent->level+1;
    else
      v->level=0;
  }
};




template <class Ops>
bool TreePoseGraph<Ops>::buildSimpleTree(){
  root=0;
  //rectify all the constraints, so that the v1<v2
  for (typename EdgeMap::iterator it=edges.begin(); it!=edges.end(); it++){
    Edge* e=it->second;
    if (e->v1->id > e->v2->id)
      revertEdge(e);
  }

  //clear the tree data
  for (typename VertexMap::iterator it=vertices.begin(); it!=vertices.end(); it++){
    Vertex* v=it->second;
    v->parent=0;
    v->parentEdge=0;
    v->children.clear();
  }

  //fill the structure
  for (typename VertexMap::iterator it=vertices.begin(); it!=vertices.end(); it++){
    Vertex* v=it->second;
    if (v->edges.empty()){
      assert(0);
      continue;
    }
    Edge* bestEdge=v->edges.front();
    int bestId=std::numeric_limits<int>::max();
    bool found=false;
    typename EdgeList::iterator li=v->edges.begin();
    while(li!=v->edges.end()){
      Edge* e =*li;
      if (e->v2==v && e->v1->id<bestId){ //consider only the entering edges
	bestId=e->v1->id;
	bestEdge=e;
	found=true;
      }
      li++;
    }
    if (found){
      v->parentEdge=bestEdge;
      v->parent=bestEdge->v1;
      v->parent->children.push_back(bestEdge);
    } else {
      assert(! root);
      root=v;
    }
  }
//   std::cerr << "root="  << root << std::endl;
  assert(root);

  //assign the level
  LevelAssigner< TreePoseGraph<Ops> > oa;
  treeDepthVisit(oa, root);

  fillEdgesInfo();
  return true;
}

template <class Ops>
TreePoseGraph<Ops>::~TreePoseGraph(){
  clear();
}

template <class Ops>
void TreePoseGraph<Ops>::clear(){
  for (typename VertexMap::iterator it=vertices.begin(); it!=vertices.end(); it++){
    delete it->second;
    it->second=0;
  }
  for (typename EdgeMap::iterator it=edges.begin(); it!=edges.end(); it++){
    delete it->second;
    it->second=0;
  }
  vertices.clear();
  edges.clear();
  if ( sortedEdges )
    delete sortedEdges;
  sortedEdges=0;
}

template <class Ops>
void TreePoseGraph<Ops>::fillEdgeInfo(Edge* e){
  Vertex* v1=e->v1;
  Vertex* v2=e->v2;
  int length=0;
  while (v1!=v2) {
    if (v1->level > v2->level){
      v1=v1->parent;
      length++;
    } else if (v2->level > v1->level){
      v2=v2->parent;
      length++;
    } else if (v1->level==v2->level){
      v1=v1->parent;
      v2=v2->parent;
      length+=2;
    }
  }
  e->length=length;
  e->top=v1;
}

template <class Ops>
void TreePoseGraph<Ops>::fillEdgesInfo(){
  typename TreePoseGraph<Ops>::EdgeMap em=edges;
  for(typename EdgeMap::iterator it=em.begin(); it!=em.end(); it++){
    fillEdgeInfo(it->second);
  }
}


template <class Ops>
typename TreePoseGraph<Ops>::EdgeSet* TreePoseGraph<Ops>::sortEdges(){
  EVComparator<Edge*> comp;
  comp.mode=edgeCompareMode;
  EdgeSet * el=new EdgeSet(comp);
  typename EdgeMap::iterator it=edges.begin();
  while(it!=edges.end()){
    el->insert(it->second);
    it++;
  }
  return el;
}

template <class Ops>
typename TreePoseGraph<Ops>::EdgeSet* TreePoseGraph<Ops>::affectedEdges(Vertex* v){
  EVComparator<Edge*> comp;
  comp.mode=edgeCompareMode;
  EdgeSet * es=new EdgeSet(comp);
  std::deque<Vertex*> frontier;
  std::list<Vertex*> markedVertices;

  //frontier.push_back(v);
  //v->mark=true;

  for (typename EdgeList::iterator it=v->children.begin(); it!=v->children.end(); it++){
    Edge* e=*it;
    Vertex* other=(e->v1==v)?e->v2:e->v1;
    frontier.push_back(other);
    other->mark=true;
    markedVertices.push_back(other);
    e->mark=true;
    es->insert(e);
   }

  while (! frontier.empty()){
    Vertex* c=frontier.front();
    frontier.pop_front();
    markedVertices.push_back(c);
    EdgeList& el=c->edges;
    for (typename EdgeList::iterator it=el.begin(); it!=el.end(); it++){
      Edge* e=*it;
      if (e->mark)
	continue;
      Vertex* other= (e->v1==c)?e->v2:e->v1;
      if (other==c->parent)
	continue;
      if (other!=e->top && ! e->top->mark){
	e->top->mark=true;
	frontier.push_back(e->top);
      }
      e->mark=true;
      es->insert(e);
      if (!other->mark){
	other->mark=true;
	frontier.push_back(other);
      }
    }
  }
  for (typename std::list<Vertex*>::iterator it=markedVertices.begin(); it!=markedVertices.end(); it++){
    (*it)->mark=false;
  }

  for (typename EdgeSet::iterator it=es->begin(); it!=es->end(); it++){
    (*it)->mark=false;
  }
  return es;
}

template <class Ops>
typename TreePoseGraph<Ops>::EdgeSet* TreePoseGraph<Ops>::affectedEdges(typename TreePoseGraph<Ops>::VertexSet& vl){
  EVComparator<Edge*> comp;
  comp.mode=edgeCompareMode;
  EdgeSet * es=new EdgeSet(comp);
  std::deque<Vertex*> frontier;
  std::list<Vertex*> markedVertices;

//   for (typename VertexSet::iterator it=vl.begin(); it!=vl.end(); it++){
//     frontier.push_back(*it);
//     (*it)->mark=true;
//   }

  for (typename VertexSet::iterator it=vl.begin(); it!=vl.end(); it++){
    Vertex* v=*it;
    for (typename EdgeList::iterator it=v->children.begin(); it!=v->children.end(); it++){
      Edge* e=*it;
      Vertex* other=(e->v1==v)?e->v2:e->v1;
      frontier.push_back(other);
      other->mark=true;
      markedVertices.push_back(other);
      e->mark=true;
      es->insert(e);
    }
  }




  while (! frontier.empty()){
    Vertex* c=frontier.front();
    frontier.pop_front();
    markedVertices.push_back(c);
    EdgeList& el=c->edges;
    for (typename EdgeList::iterator it=el.begin(); it!=el.end(); it++){
      Edge* e=*it;
      if (e->mark)
	continue;
      Vertex* other= (e->v1==c)?e->v2:e->v1;
      if (other==c->parent)
	continue;
      if (other!=e->top && ! e->top->mark){
	e->top->mark=true;
	frontier.push_back(e->top);
      }
      e->mark=true;
      es->insert(e);
      if (!other->mark){
	other->mark=true;
	frontier.push_back(other);
      }
    }
  }
  for (typename std::list<Vertex*>::iterator it=markedVertices.begin(); it!=markedVertices.end(); it++){
    (*it)->mark=false;
  }

  for (typename EdgeSet::iterator it=es->begin(); it!=es->end(); it++){
    (*it)->mark=false;
  }
  return es;
}



template <class Ops>
int TreePoseGraph<Ops>::maxPathLength(){
  int max=0;
  typename EdgeMap::const_iterator it=edges.begin();
  while(it!=edges.end()){
    int l=it->second->length;
    max=l>max?l:max;
    it++;
  }
  return max;
}

template <class Ops>
int TreePoseGraph<Ops>::totalPathLength(){
  int t=0;
  typename EdgeMap::const_iterator it=edges.begin();
  while(it!=edges.end()){
    t+=it->second->length;
    it++;
  }
  return t;
}


template <class Ops>
void TreePoseGraph<Ops>::compressIndices(){
  VertexMap vmap;
  int i=0;
  for (typename VertexMap::iterator it=vertices.begin(); it!=vertices.end(); it++){
    Vertex* v=it->second;
    v->id=i;
    vmap.insert(std::make_pair(i,v));
    i++;
  }
  vertices=vmap;
}

template <class Ops>
int TreePoseGraph<Ops>::maxIndex(){
  typename VertexMap::reverse_iterator it=vertices.rbegin();
  if (it!=vertices.rend())
    return it->second->id;
  return -1;
}



template <class TPG>
struct LoopChecker{
  bool noloops;
  void perform(typename TPG::Vertex* v){
    if (!noloops)
      return;
    if (!v->mark)
      v->mark=true;
    else
      noloops=false;
  }
};


template <class Ops>
bool TreePoseGraph<Ops>::sanityCheck(){
  //check that each node has exactly one parent
  for (typename VertexMap::iterator it=vertices.begin(); it!=vertices.end(); it++){
    Vertex* v=it->second;
    v->mark=false;
    Vertex* vp=v->parent;

    if (! vp){
      if (v!=root){
	std::cerr << "root not found in the graph" << std::endl;
	return false;
      }
    }

    const EdgeList& children=it->second->children;
    for (typename EdgeList::const_iterator lt=children.begin(); lt!=children.end(); lt++){
      if ((*lt)->v1!=v){
	std::cerr << "wrong direction of the edges" << std::endl;
	return false;
      }
    }
  }
  //check that there are no loops in the tree

  LoopChecker< TreePoseGraph<Ops> > lc;
  lc.noloops=true;
  treeBreadthVisit(lc);
  if (!lc.noloops){
    std::cerr << "the tree contains loops" << std::endl;
    return false;
  }
  for (typename VertexMap::iterator it=vertices.begin(); it!=vertices.end(); it++){
    Vertex* v=it->second;
    v->mark=false;
  }
  return true;
}
