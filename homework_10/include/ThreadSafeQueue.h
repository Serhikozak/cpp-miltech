#pragma once
#include <queue>
#include <mutex>

template <typename T>
class ThreadSafeQueue {
    private:
        std::queue<T> m_queue;
        mutable std::mutex m_mutex;

    public:
        ThreadSafeQueue() = default;
        ~ThreadSafeQueue() = default;

        //Додавання елемента в чергу
        void push(const T& value) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_queue.push(value);
        }

        bool empty() const {
            std::lock_guard<std::mutex> lock(m_mutex);
            return m_queue.empty();
        }

        bool try_pop(T& value) {
            std::lock_guard<std::mutex> lock(m_mutex);
            if (m_queue.empty()) {
                return false;
            }
            value = m_queue.front();
            m_queue.pop();
            return true;

        }

};