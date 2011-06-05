/*
 * Copyright (C) 2010-2011, Mathieu Labbe and IntRoLab - Universite de Sherbrooke
 *
 * This file is part of RTAB-Map.
 *
 * RTAB-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * RTAB-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with RTAB-Map.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef NODE_H_
#define NODE_H_

namespace rtabmap {

class Node
{
public:
	Node(int id, Node * parent = 0) :
		_parent(parent),
		_id(id)
	{
		if(_parent)
		{
			_parent->addChild(this);
		}
	}
	virtual ~Node()
	{
		//We copy the set because when a child is destroyed, it is removed from its parent.
		std::set<Node*> children = _children;
		_children.clear();
		for(std::set<Node*>::iterator iter=children.begin(); iter!=children.end(); ++iter)
		{
			delete *iter;
		}
		children.clear();
		if(_parent)
		{
			_parent->removeChild(this);
		}
	}
	int id() const {return _id;}
	bool isAncestor(int id) const
	{
		if(_parent)
		{
			if(_parent->id() == id)
			{
				return true;
			}
			return _parent->isAncestor(id);
		}
		return false;
	}

	void expand(std::list<std::list<int> > & paths, std::list<int> currentPath = std::list<int>()) const
	{
		currentPath.push_back(_id);
		if(_children.size() == 0)
		{
			paths.push_back(currentPath);
			return;
		}
		for(std::set<Node*>::const_iterator iter=_children.begin(); iter!=_children.end(); ++iter)
		{
			(*iter)->expand(paths, currentPath);
		}
	}

private:
	void addChild(Node * child)
	{
		_children.insert(child);
	}
	void removeChild(Node * child)
	{
		_children.erase(child);
	}

private:
	std::set<Node*> _children;
	Node * _parent;
	int _id;
};

}

#endif /* NODE_H_ */
