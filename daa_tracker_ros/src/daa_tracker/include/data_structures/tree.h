/*
 * Copyright (c) 2023, DFKI GmbH and contributors
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *    * Neither the name of the the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#include <list>
#include <memory>
#include <functional>
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <cassert>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/iteration_macros.hpp>

using namespace std;
namespace daa
{
namespace data_structures
{
template <class T>
class TreeNode;

template <class T>
class TreeUtils;

template <class T>
class TreeNode : public enable_shared_from_this<TreeNode<T>>
{
  friend class TreeUtils<T>;

public:
  typedef shared_ptr<TreeNode> Ptr;
  typedef weak_ptr<TreeNode> wPtr;
  static TreeNode<T>::Ptr create()
  {
    return TreeNode<T>::Ptr();
  }
  static TreeNode<T>::Ptr create(const T& item, TreeNode<T>::Ptr parent = nullptr)
  {
    return make_shared<TreeNode<T>>(parent, item);
  }

  TreeNode<T>::Ptr expand(const T& item)
  {
    auto child = create(item, this->shared_from_this());
    children_.push_back(child);
    return child;
  }
  void validate()
  {
    valid_flag_ = true;
  }
  void invalidate()
  {
    valid_flag_ = false;
  }
  bool is_valid()
  {
    return valid_flag_;
  }
  T& item()
  {
    return item_;
  }
  size_t depth() const
  {
    return depth_;
  }
  TreeNode<T>::Ptr parent()
  {
    return parent_.lock();
  }
  //! for debugging
  list<TreeNode<T>::Ptr> children()
  {
    return children_;
  }

  TreeNode() = delete;
  TreeNode(TreeNode<T>::Ptr parent, const T& item) : item_(item), parent_(parent), valid_flag_(false)
  {
    if (parent_.lock())
      depth_ = parent->depth() + 1;
    else
      depth_ = 0;
  }
  ~TreeNode()
  {
    for (auto& node : children_)
      node = nullptr;
  }

private:
  bool valid_flag_;
  T item_;
  size_t depth_;
  TreeNode<T>::wPtr parent_;
  list<TreeNode<T>::Ptr> children_;
};

template <class T>
class TreeUtils
{
public:
  TreeUtils() = delete;
  ~TreeUtils() = delete;
  using TreeFunctionType = function<void(class TreeNode<T>::Ptr&)>;

  static class TreeNode<T>::Ptr n_scan_pruning(class TreeNode<T>::Ptr leaf, size_t n)
  {
    class TreeNode<T>::Ptr root = nullptr;
    if (leaf != nullptr && n >= 2)
    {
      root = get_Nth_ancestor(leaf, n - 1);              // the deepest one within the N-scan window
      if (root == nullptr || root->parent() == nullptr)  // tree depth <= N
        return nullptr;
      // cut relation to the parent
      auto it_root = find(root->parent()->children_.begin(), root->parent()->children_.end(), root);
      root->parent()->children_.erase(it_root);
      // clean the branch
      delete_branch(root->parent());
      // resent depth
      set_depth(root, 0);
    }
    return root;
  } static void merge_roots(class TreeNode<T>::Ptr& root_to_keep, class TreeNode<T>::Ptr& root)
  {
    for (auto node : root->children_)
    {
      root_to_keep->children_.push_back(node);
      node->parent_ = root_to_keep;
    }
    root->children_.clear();
  }
  static void prune_invalid(class TreeNode<T>::Ptr& root)
  {
    postorder_traverse(root, delete_invalid_func);
  }

  static void delete_branch(class TreeNode<T>::Ptr root)
  {
    postorder_traverse(root, delete_func);
  }

  static void validate_branch(class TreeNode<T>::Ptr leaf)
  {
    reverse_order_traverse(leaf, mark_valid_func);
  }

  static void invalidate_branch(class TreeNode<T>::Ptr& root)
  {
    postorder_traverse(root, mark_invalid_func);
  }

  static vector<class TreeNode<T>::Ptr> get_leaves(class TreeNode<T>::Ptr& root)
  {
    vector<class TreeNode<T>::Ptr> ret;
    if (root != nullptr)
    {
      if (root->children_.empty())
      {
        // vector<class TreeNode<T>::Ptr> result;
        ret.push_back(root);
        return ret;
      }
      else
      {
        for (auto child : root->children_)
        {
          auto nodes = get_leaves(child);
          ret.insert(ret.end(), nodes.begin(), nodes.end());
        }
      }
    }
    return ret;
  }

  static void
  set_depth(class TreeNode<T>::Ptr& root, size_t depth = 0)
  {
    if (root != nullptr)
    {
      root->depth_ = depth;
      for (auto& child : root->children_)
      {
        set_depth(child, depth + 1);
      }
    }
  }
  /**
   *   0 1 2 ... Nth ancestor
   *   o o o ... o
   */
  static class TreeNode<T>::Ptr get_Nth_ancestor(class TreeNode<T>::Ptr node, size_t n)
  {
    if (node != nullptr)
    {
      if (n > 0)
        return get_Nth_ancestor(node->parent(), --n);
      else
        return node;
    }
    else
    {
      // auto null_ptr = TreeNode<T>::create();
      return nullptr;
    }

  }

  static class TreeNode<T>::Ptr
  get_root(class TreeNode<T>::Ptr node)
  {
    if (node->parent())
    {
      return get_root(node->parent());
    }
    else
      return node;
  }
  // template<class T> //https://stackoverflow.com/questions/5524744/typedef-inside-template-class-doesnt-work
  template <typename F>
  static void postorder_traverse(class TreeNode<T>::Ptr& root, F func)
  {
    if (root != nullptr)
    {
      for (auto it_child = root->children_.begin(); it_child != root->children_.end(); it_child++)
      {
        postorder_traverse(*it_child, func);
      }
      func(root);
    }
  }
  template <typename F, typename... Args>
  static void preorder_traverse(class TreeNode<T>::Ptr& root, F func, Args... args)
  {
    if (root != nullptr)
    {
      func(root, args...);
      for (auto it_child = root->children_.begin(); it_child != root->children_.end(); it_child++)
      {
        preorder_traverse(*it_child, func, args...);
      }
    }
  }
  template <typename F>
  static void reverse_order_traverse(class TreeNode<T>::Ptr leaf_node, F func)
  {
    if (leaf_node != nullptr)
    {
      func(leaf_node);
      reverse_order_traverse(leaf_node->parent(), func);
    }
  }

  static TreeFunctionType delete_func;
  static TreeFunctionType delete_invalid_func;
  static TreeFunctionType mark_valid_func;
  static TreeFunctionType mark_invalid_func;

  // visualizing
  class Viz
  {
  private:
    struct Vertex
    {
      string label;
      double score;
    };
    struct Edge
    {
      double weight;
      double max_score;
    };
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, Vertex, Edge> Graph;
    typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_t;
    typedef typename boost::graph_traits<Graph>::edge_descriptor edge_t;
    typedef function<void(class TreeNode<T>::Ptr, Graph*, vertex_t&)> VisitFuncType;

    typedef function<string(T)> LabelFuncType;
    typedef function<double(T)> WeightFuncType;

  private:
    // static const VisitFuncType visit_node;
  public:
    static void generate_graph(class TreeNode<T>::Ptr root, const std::string& file_name, LabelFuncType label_func,
                               WeightFuncType weight_func)
    {
      Graph* g = new Graph();
      VisitFuncType visit_node = [&label_func, &weight_func](class TreeNode<T>::Ptr node, Graph* g,
                                                             vertex_t& vertex = vertex_t()) {
        if (node)
        {
          auto& graph = *g;
          vertex_t v = add_vertex(graph);
          if (node->parent())
          {
            edge_t e = add_edge(vertex, v, graph).first;
            // graph[e].weight = node->item()->score;
          }
          graph[v].label = label_func(node->item());

          vertex = v;
        }
      };

      preorder_traverse(root, visit_node, g, vertex_t());
      std::ofstream dot_file(file_name.c_str());
      write_graphviz(dot_file, *g, make_label_writer(boost::get(&Vertex::label, *g)));

      delete g;
    }
  };
};

template <class T>
class TreeUtils<T>::TreeFunctionType TreeUtils<T>::delete_func = [](class TreeNode<T>::Ptr& node)
{
  if (node != nullptr)
  {
    // cout << "@[" << node << "] delete_func: deleting item " << node->item_ << ", use count: " << node.use_count()
    //  << endl;  // TODO: for debug
    node->children_.clear();
    node = nullptr;
  }
};

template <class T>
class TreeUtils<T>::TreeFunctionType TreeUtils<T>::delete_invalid_func = [](class TreeNode<T>::Ptr& node)
{
  if (node != nullptr)
  {
    // node->parent_->children_;
    if (!node->is_valid())
    {
      // cout << "delete_invalid_func: deleting" << node->item_ << endl;  // TODO: for debug
      node->children_.clear();
      node = nullptr;
    }
    else  // delete nullptr children
    {
      for (auto it = node->children_.begin(); it != node->children_.end();)
      {
        if (*it)  // not nullptr
          it++;
        else
          it = node->children_.erase(it);
      }
    }
  }
};

template <class T>
class TreeUtils<T>::TreeFunctionType TreeUtils<T>::mark_valid_func = [](class TreeNode<T>::Ptr& node)
{
  if (node != nullptr)
  {
    node->validate();
  }
};

template <class T>
class TreeUtils<T>::TreeFunctionType TreeUtils<T>::mark_invalid_func = [](class TreeNode<T>::Ptr& node)
{
  if (node != nullptr)
  {
    node->invalidate();
  }
};

}  // end namespace data_structures

// template <class T>
// class TreeMonitor
// {
// public:
//   TreeMonitor()
//   {
//     run_thread_ = thread(&TreeMonitor<T>::run, this);
//     stop_ = true;
//     tree_ = nullptr;
//   }
//   ~TreeMonitor()
//   {
//     if (run_thread_.joinable())
//       run_thread_.join();
//   }

//   void set_tree(class data_structures::TreeNode<T>::Ptr tree)
//   {
//     tree_ = tree;
//   }
//   void start()
//   {
//     stop_ = false;
//   }

// private:
//   void run()
//   {
//     while (!stop_)
//     {
//       cout << "tree monitor runing..." << endl;
//       if (tree_)
//       {
//         cout << "tree has " << data_structures::TreeUtils<T>::get_leaves(tree_).size() << " leaves" << endl;
//       }
//       this_thread::sleep_for(2000ms);
//     }
//   }
//   // vector<class data_structures::TreeNode<T>::Ptr> trees_;
//   atomic_bool stop_;
//   class data_structures::TreeNode<T>::Ptr tree_;
//   thread run_thread_;
// };

}  // namespace daa
