#pragma once
#include <functional>

namespace Collision
{
	namespace DataStructures
	{
		template<typename T>
		struct ListNode
		{
			ListNode<T> *m_prev, *m_next;
			T m_data;
		};

		template <typename T>
		class LinkedList
		{
		public:

			LinkedList<T>();
			~LinkedList<T>();

			bool IsEmpty() const;
			size_t GetSize() const;
			void InsertBack(T data);
			void Remove(T data);
			
			ListNode<T> *m_head, *m_tail;
		private:

			size_t m_size;

			void InsertBetween(ListNode<T> *toBeAdded, ListNode<T> *prev, ListNode<T> *next);
			void __Remove(ListNode<T> *node);
		};

		template <typename T>
		Collision::DataStructures::LinkedList<T>::LinkedList()
		{
			m_size = 0;
			m_head = m_tail = NULL;
		}

		template <typename T>
		Collision::DataStructures::LinkedList<T>::~LinkedList()
		{
			ListNode<T> *crawler = m_head;

			while (crawler)
			{
				if (crawler->m_prev)
					delete crawler->m_prev;

				crawler = crawler->m_next;
			}
		}

		template <typename T>
		bool Collision::DataStructures::LinkedList<T>::IsEmpty() const
		{
			return !m_size;
		}

		template <typename T>
		size_t Collision::DataStructures::LinkedList<T>::GetSize() const
		{
			return m_size;
		}

		template <typename T>
		void Collision::DataStructures::LinkedList<T>::InsertBack(T data)
		{
			ListNode<T> *newNode = new ListNode<T>();
			if (IsEmpty())
			{
				newNode->m_next = newNode->m_prev = NULL;
				newNode->m_data = data;
				m_head = m_tail = newNode;
			}
			else
			{
				newNode->m_data = data;
				newNode->m_prev = m_tail;
				newNode->m_next = NULL;
				m_tail->m_next = newNode;
				m_tail = newNode;
			}

			m_size++;
			std::wcout << "LIST [" << m_size << "] addrs: ";
			for (auto asd = m_head; asd; asd = asd->m_next)
			{
				std::wcout << asd << " ";
			}
			std::wcout << std::endl;
		}

		template <typename T>
		void Collision::DataStructures::LinkedList<T>::InsertBetween(ListNode<T> *toBeAdded, ListNode<T> *prev, ListNode<T> *next)
		{
			if (prev)
			{
				prev->m_next = toBeAdded;
				toBeAdded->m_prev = prev;
			}

			if (next)
			{
				next->m_prev = toBeAdded;
				toBeAdded->m_next = next;
			}

			m_size++;
		}

		template <typename T>
		void Collision::DataStructures::LinkedList<T>::Remove(T data)
		{
			ListNode<T> *crawler = m_head;

			while (1)
			{
				if (crawler->m_data == data)
					__Remove(crawler);

				crawler = crawler->m_next;
				if (crawler == m_head)
				{
					break;
				}
			}
		}

		template <typename T>
		void Collision::DataStructures::LinkedList<T>::__Remove(ListNode<T> *node)
		{			
			if (!node)
				return;

			if (node->m_prev)
			{
				node->m_prev->m_next = node->m_next;
			}

			if (node->m_next)
			{
				node->m_next->m_prev = node->m_prev;
			}

			delete node;
			size--;			
		}

	}
}

