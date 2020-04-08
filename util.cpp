#ifndef UTIL
#define UTIL
#include <iostream>
#include <cstdlib>
#include <vector>
using namespace std;

/* miscellaneous */
inline void seperator(int length=20){cout << string(length,'-') << endl;}
inline double prob(){return static_cast<double>(rand())/RAND_MAX;}
inline double uniformRand(double min, double max){return min+(max-min)*prob();}

/******************************************************************************/
/* print vector */
template <class T>
ostream& operator<<(ostream& out, const vector<T>& vec){
    // print elements of a vector
    if(vec.size()==0) out << "[ ]";
    else{
        out << "[ ";
        for(int i=0; i<vec.size()-1; i++) out << vec[i] << ", ";
        out << vec[vec.size()-1] << " ]";
    }
    return out;
}

/******************************************************************************/
/* priority queue */
struct node{
    // queue element
    int label;
    double value;
    node* next;
    node(int label, double value):label(label),value(value),next(0){}
};

inline ostream& operator<<(ostream& out, const node& n){
    out << '(' << n.label << ',' << n.value << ')';
    return out;
}

class priorityQueue{
private:
    node* head;
public:
    /**** constructors ****/
    priorityQueue();
    /**** destructors ****/
    // ~priorityQueue();
    /**** accessors ****/
    int getSize();
    bool isEmpty();
    bool contains(int label);
    node peekHeadNode();
    node peekTailNode();
    void print();
    /**** mutators ****/
    void add(int label, double value);
    void add(node n);
    void del(int label);
    void chgPriority(int label, double value);
    node popHeadNode();
};

/**** constructors ****/

priorityQueue::priorityQueue():head(0){}

/**** accessors ****/

int priorityQueue::getSize(){
    // number of nodes in queue
    int size = 0;
    node* p = head;
    while(p!=0){
        size++;
        p = p->next;
    }
    return size;
}

bool priorityQueue::isEmpty(){
    // check if queue is empty
    if(head==0) return true;
    return false;
}

bool priorityQueue::contains(int label){
    // check if queue contains node as labelled
    node* p = head;
    while(p!=0){
        if(p->label==label) return true;
        p = p->next;
    }
    return false;
}

node priorityQueue::peekHeadNode(){
    // return a copy of head node
    node n(head->label,head->value);
    return n;
}

node priorityQueue::peekTailNode(){
    // return a copy of tail node
    node* p = head;
    while(p->next!=0) p = p->next;
    node n(p->label,p->value);
    return n;
}

void priorityQueue::print(){
    // print whole queue
    node* p = head;
    while(p!=0){
        cout << *p << " -> ";
        p = p->next;
    }
    cout << "###" << endl;
}

/**** mutators ****/

void priorityQueue::add(int label, double value){
    // add new node (label,value)
    node* n = new node(label,value);
    node* p;
    if(head==0 || value>head->value){
        // empty queue or higher priority than head node
        n->next = head;
        head = n;
    }else{
        p = head;
        // stop at node with priority just higher than n->value
        while(p->next!=0 && p->next->value>=value)
            p = p->next;
        n->next = p->next;
        p->next = n;
    }
}

void priorityQueue::add(node n){
    // add new node
    add(n.label,n.value);
}

void priorityQueue::del(int label){
    // delete node as labelled
    node* p = head;
    node* prev;
    if(contains(label)){
        while(p->label!=label){
            prev = p;
            p = p->next;
        }
        if(p==head) head = head->next;
        else prev->next = p->next;
        free(p);
    }
}

void priorityQueue::chgPriority(int label, double value){
    // change priority of node as labelled
    del(label);
    add(label,value);
}

node priorityQueue::popHeadNode(){
    // pop and return a copy of head node
    node* p = head;
    head = head->next;
    node n(p->label,p->value);
    free(p);
    return n;
}

#endif
