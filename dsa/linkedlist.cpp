/*
 * (c) 2025 Yunjae Lim <launius@gmail.com>
 *
 * C++ Linked List Implementation
 *
 */

#include <iostream>

using namespace std;

struct Node {
	int data;
	Node* next;
};

class LinkedList {
	Node* head;
	
public:
	LinkedList() : head(NULL) {
		cout << __func__ << ": sizeof(Node) " << sizeof(Node) << endl;
	}

	void traverse() {
		Node* current = head;
		while (current) {
			cout << "(" << current << ") " << current->data << endl;
			current = current->next;
		}

		cout << "----------" << endl;
	}
	
	void insertAtBeginning(int data) {
		cout << __func__ << ": " << data << endl;

		Node* newNode = new Node;
		newNode->data = data;
		newNode->next = head;
		
		head = newNode;
	}
	
	void insertAtEnd(int data) {
		cout << __func__ << ": " << data << endl;

		Node* newNode = new Node;
		newNode->data = data;
		newNode->next = NULL;

		if (!head) {
			head = newNode;
			return;
		}

		Node* temp = head;
		while(temp->next)
			temp = temp->next;
		temp->next = newNode;
	}
	
	void insertAtPosition(int data, int pos) {
		cout << __func__ << ": " << data << " at " << pos << endl;

		if (pos < 1)
			return;
		
		if (pos == 1) {
			insertAtBeginning(data);
			return;
		}

		Node* prev = head;	// previous node of insertion
		while (pos - 2 && prev) {
			prev = prev->next;
			pos--;
		}
		
		if (!prev) {
			cout << __func__ << ": out of range!" << endl;
			return;
		}

		Node* newNode = new Node;
		newNode->data = data;
		newNode->next = prev->next;
		
		prev->next = newNode;
	}

	void deletefromBeginning() {
		cout << __func__ << ": " << endl;

		if (!head)
			return;

		Node* temp = head;
		head = head->next;
		delete temp;
	}
	
	void deletefromEnd() {
		cout << __func__ << ": " << endl;

		if (!head)
			return;

		if (!head->next) {
			delete head;
			head = NULL;
			return;
		}
		
		Node* secondLast = head;
		while (secondLast->next->next)
			secondLast = secondLast->next;
		delete secondLast->next;
		secondLast->next = NULL;
	}
	
	void deleteAtPosition(int pos) {
		cout << __func__ << ": at " << pos << endl;

		if (pos < 1)
			return;

		if (pos == 1) {
			deletefromBeginning();
			return;
		}

		Node* prev = head;	// previous node of deletion
		while (pos - 2 && prev) {
			prev = prev->next;
			pos--;
		}

		if (!prev || !prev->next) {
			cout << __func__ << ": out of range! " << endl;
			return;
		}

		Node* temp = prev->next;
		prev->next = prev->next->next;
		delete temp;
	}
};

int main()
{
	LinkedList list;
	list.traverse();

	list.insertAtEnd(10);
	list.insertAtEnd(20);
	list.insertAtEnd(30);
	list.insertAtBeginning(5);
	list.insertAtBeginning(1);
	list.traverse();

	list.insertAtPosition(15, 4);
	list.traverse();

	list.deletefromBeginning();
	list.traverse();

	list.deletefromEnd();
	list.traverse();

	list.deleteAtPosition(4);
	list.traverse();

	return 0;
}
