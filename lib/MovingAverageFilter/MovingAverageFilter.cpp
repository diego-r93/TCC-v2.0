#include "MovingAverageFilter.h"

MovingAverageFilter::MovingAverageFilter(int size) : _size(size), _index(0), _sum(0) {
   _buffer = new int[_size];
   for (int i = 0; i < _size; ++i) {
      _buffer[i] = 0;  // Inicializa o buffer com zeros
   }
}

MovingAverageFilter::~MovingAverageFilter() {
   delete[] _buffer;  // Libera o buffer
}

void MovingAverageFilter::addValue(int value) {
   _sum -= _buffer[_index];        // Subtrai o valor antigo do somatório
   _buffer[_index] = value;        // Atualiza o buffer com o novo valor
   _sum += value;                  // Adiciona o novo valor ao somatório
   _index = (_index + 1) % _size;  // Atualiza o índice (circular)
}

float MovingAverageFilter::getAverage() const {
   return static_cast<float>(_sum) / _size;  // Calcula e retorna a média
}
