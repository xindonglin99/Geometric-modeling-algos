#pragma once

#include <algorithm>
#include <vector>

//===================================== Bit_vector ===============================
// Essentially a bit vector with an efficient means for clearing all bits
// Thanks to Robert Bridson

namespace minimesh
{

	struct Bit_vector
	{
		explicit Bit_vector(unsigned int n = 0)
				:flag(n, 0), counter(1)
		{
		}

		void clear(void)
		{
			flag.clear();
			counter = 1;
		}

		void resize(unsigned int n_)
		{
			flag.resize(n_, 0);
		}

		void reset_all(void)
		{
			++counter;
			if (counter == 0)
			{ // if we hit wrap-around, reset everything
				for (unsigned int i = 0; i < flag.size(); ++i)
					flag[i] = 0;
				counter = 1;
			}
		}

		bool operator[](unsigned int i) const
		{
			return flag[i] == counter;
		}

		void set(unsigned int i)
		{
			flag[i] = counter;
		}

		unsigned int size()
		{
			return flag.size();
		}

		void swap(Bit_vector& othermarker)
		{
			std::swap(flag, othermarker.flag);
			std::swap(counter, othermarker.counter);
		}

	protected:
		std::vector<unsigned int> flag;
		unsigned int counter;
	};

} // end of minimesh
