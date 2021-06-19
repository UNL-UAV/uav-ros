#pragma once

template<typename T = uint8_t>
class State{
private:
	T _state;
public:
	State() = default;
	inline T getState() const { return _state;};
	inline void setState(T state) { this->_state = state; };
	~State() = default;
private:
};