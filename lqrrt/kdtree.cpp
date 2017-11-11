#define WRAP_PYTHON 1
#if WRAP_PYTHON
#include <Python.h>
#include <numpy/arrayobject.h>
#include <boost/python.hpp>
#endif

/*
The Great KD Tree of Path Planning
        Creator: Nicholas Suhlman
        Maintainer: Nicholas Suhlman
        Email: Suhlman3@gmail.com
*/
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

using namespace Eigen;
using namespace std;

class Tree
{
public:
  int nstates;
  MatrixXd state;
  int size;
  int pID;
  vector<int> pIDs;
  vector<MatrixXd> lqr, x_seq, u_seq;
  vector<MatrixXd> ncontrols;

  int add_node(int pID, MatrixXd state, MatrixXd lqr, MatrixXd x_seq, MatrixXd u_seq)
  {
    // Make sure the desired parent exists
    if (pID >= this->size || pID < 0)
    {
      // raise ValueError("The given parent ID, {}, doesn't exist".format(pID));
      return -1;
    }

    // Update State Array
    this->state(this->state.rows() + state.rows(), this->state.rows());

    // Append all otjer feature lists
    this->pIDs.push_back(pID);
    this->lqr.push_back(lqr);
    this->x_seq.push_back(x_seq);
    this->u_seq.push_back(u_seq);

    // Increment node count
    this->size += 1;
    return 0;
  }

  vector<int> climb(int ID)
  {
    // Make sure the desired end node exists
    if (ID >= this->size or ID < 0)
    {
      // raise ValueError("The given ID, {}, doesn't exist.".format(ID));
    }
    vector<int> IDs;
    while (ID != -1)
    {
      IDs.push_back(ID);
      ID = this->pIDs[ID];
    }
    return IDs;
  }

  vector<vector<MatrixXd>> trajectory(vector<int> IDs)
  {
    vector<MatrixXd> x_seq_full, u_seq_full;
    for (int i = 0; i < IDs.size(); i++)
    {
      x_seq_full.push_back(this->x_seq[i]);
      u_seq_full.push_back(this->u_seq[i]);
    }
    vector<vector<MatrixXd>> rvec;
    rvec.push_back(x_seq_full);
    rvec.push_back(u_seq_full);
    return (rvec);
  }

  template <typename Derived>
  Tree(const MatrixBase<Derived>& seed_state, const MatrixBase<Derived>& seed_lqr)
  {
    nstates = seed_state.size();

    try
    {
      ncontrols.push_back(seed_lqr);
    }
    catch (const bad_alloc e)
    {
      printf("\nThe given seed_lqr is not consistent.");
      printf("Continuing, assuming you don't care about the lqr or effort "
             "features...\n");
    }
    MatrixXd state(3, 3);
    state = seed_state;
    state.resize(1, nstates);
    pID = -1;

    lqr.push_back(seed_lqr);
    x_seq.push_back(seed_state);
    MatrixXd temp = MatrixXd::Zero(1, ncontrols.size());
    u_seq.push_back(temp);

    size = 1;
  }

  // #if WRAP_PYTHON
  //   // This code is pretty 'treepy' aha aha aha
  //   Tree(PyObject* seed_state_py, PyObject* seed_lqr_py, int seed_size)
  //   {
  //     nstates = seed_size;

  //     try
  //     {
  //       ncontrols.push_back(seed_lqr_py);
  //     }
  //     catch (const bad_alloc e)
  //     {
  //       printf("\nThe given seed_lqr is not consistent.");
  //       printf("Continuing, assuming you don't care about the lqr or effort "
  //              "features...\n");
  //     }
  //     Map<MatrixXd> state(seed_state_py->data(), 1, nstates);
  //     pID = -1;

  //     lqr.push_back(seed_lqr_py);
  //     x_seq.push_back(seed_state_py);
  //     MatrixXd temp = MatrixXd::Zero(1, ncontrols.size());
  //     u_seq.push_back(temp);

  //     size = 1;
  //   }
  // #endif
};
template <typename Derived>

#if WRAP_PYTHON
Tree Treepy(PyObject* seed_state_py, PyObject* seed_lqr_py)
{
  int state1 = (int)PyArray_DIM(seed_state_py, 0);
  // int state2 = (int)PyArray_DIM(seed_state_py, 1);
  MatrixXd seed_state((double*)PyArray_DATA(seed_state_py), state1);
  int lqr1 = (int)PyArray_DIM(seed_lqr_py, 0);
  // int lqr2 = (int)PyArray_DIM(seed_lqr_py, 1);
  MatrixXd seed_lqr((double*)PyArray_DATA(seed_lqr_py), lqr1);

  Tree zed = Tree(seed_state, seed_lqr);
  return zed;
}
using namespace boost::python;
BOOST_PYTHON_MODULE(_Tree)
{
  class_<Tree>("Tree", init<MatrixXd, MatrixXd>(args("seed_state", "seed_lqr")))
      // .def("Tree", &Tree::Tree)
      .def("add_node", &Tree::add_node)
      .def("trajectory", &Tree::trajectory)
      .def("climb", &Tree::climb)
      .def_readonly("nstates", &Tree::nstates)
      .def_readonly("state", &Tree::state)
      .def_readonly("size", &Tree::size)
      .def_readonly("pID", &Tree::pID)
      .def_readonly("pIDs", &Tree::pIDs)
      .def_readonly("x_seq", &Tree::x_seq)
      .def_readonly("u_seq", &Tree::u_seq)
      .def_readonly("ncontrols", &Tree::ncontrols)
      .def_readonly("lqr", &Tree::lqr);

  int nstates;
  MatrixXd state;
  int size;
  int pID;
  vector<int> pIDs;
  vector<MatrixXd> lqr, x_seq, u_seq;
  vector<MatrixXd> ncontrols;
}
#endif