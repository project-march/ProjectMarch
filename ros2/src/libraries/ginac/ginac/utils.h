/** @file utils.h
 *
 *  Interface to several small and furry utilities needed within GiNaC but not
 *  of any interest to the user of the library. */

/*
 *  GiNaC Copyright (C) 1999-2023 Johannes Gutenberg University Mainz, Germany
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef GINAC_UTILS_H
#define GINAC_UTILS_H

#include "assertion.h"

#include <functional>
#include <cstdint> // for uintptr_t
#include <string>

namespace GiNaC {

/** Exception class thrown by functions to signal unimplemented functionality
 *  so the expression may just be .hold() */
class dunno {};

// some compilers (e.g. cygwin) define a macro log2, causing confusion
#ifdef log2
#undef log2
#endif

unsigned log2(unsigned n);

/** Rotate bits of unsigned value by one bit to the left.
  * This can be necessary if the user wants to define its own hashes. */
inline unsigned rotate_left(unsigned n)
{
	return (n & 0x80000000U) ? (n << 1 | 0x00000001U) : (n << 1);
}

/** Compare two pointers (just to establish some sort of canonical order).
 *  @return -1, 0, or 1 */
template <class T>
inline int compare_pointers(const T * a, const T * b)
{
	// '<' is not defined for pointers that don't point to the same array,
	// but std::less is.
	if (std::less<const T *>()(a, b))
		return -1;
	else if (std::less<const T *>()(b, a))
		return 1;
	return 0;
}

/** Truncated multiplication with golden ratio, for computing hash values. */
inline unsigned golden_ratio_hash(uintptr_t n)
{
	return n * UINT64_C(0x4f1bbcdd);
}

/* Compute the sign of a permutation of a container, with and without an
   explicitly supplied comparison function. If the sign returned is 1 or -1,
   the container is sorted after the operation. */
template <class It>
int permutation_sign(It first, It last)
{
	using std::swap;
	if (first == last)
		return 0;
	--last;
	if (first == last)
		return 0;
	It flag = first;
	int sign = 1;

	do {
		It i = last, other = last;
		--other;
		bool swapped = false;
		while (i != first) {
			if (*i < *other) {
				swap(*other, *i);
				flag = other;
				swapped = true;
				sign = -sign;
			} else if (!(*other < *i))
				return 0;
			--i;
			if (i != first)
				--other;
		}
		if (!swapped)
			return sign;
		++flag;
		if (flag == last)
			return sign;
		first = flag;
		i = first; other = first;
		++other;
		swapped = false;
		while (i != last) {
			if (*other < *i) {
				swap(*i, *other);
				flag = other;
				swapped = true;
				sign = -sign;
			} else if (!(*i < *other))
				return 0;
			++i;
			if (i != last)
				++other;
		}
		if (!swapped)
			return sign;
		last = flag;
		--last;
	} while (first != last);

	return sign;
}

template <class It, class Cmp, class Swap>
int permutation_sign(It first, It last, Cmp comp, Swap swapit)
{
	if (first == last)
		return 0;
	--last;
	if (first == last)
		return 0;
	It flag = first;
	int sign = 1;

	do {
		It i = last, other = last;
		--other;
		bool swapped = false;
		while (i != first) {
			if (comp(*i, *other)) {
				swapit(*other, *i);
				flag = other;
				swapped = true;
				sign = -sign;
			} else if (!comp(*other, *i))
				return 0;
			--i;
			if (i != first)
				--other;
		}
		if (!swapped)
			return sign;
		++flag;
		if (flag == last)
			return sign;
		first = flag;
		i = first; other = first;
		++other;
		swapped = false;
		while (i != last) {
			if (comp(*other, *i)) {
				swapit(*i, *other);
				flag = other;
				swapped = true;
				sign = -sign;
			} else if (!comp(*i, *other))
				return 0;
			++i; 
			if (i != last)
				++other;
		}
		if (!swapped)
			return sign;
		last = flag;
		--last;
	} while (first != last);

	return sign;
}

/* Implementation of shaker sort, only compares adjacent elements. */
template <class It, class Cmp, class Swap>
void shaker_sort(It first, It last, Cmp comp, Swap swapit)
{
	if (first == last)
		return;
	--last;
	if (first == last)
		return;
	It flag = first;

	do {
		It i = last, other = last;
		--other;
		bool swapped = false;
		while (i != first) {
			if (comp(*i, *other)) {
				swapit(*other, *i);
				flag = other;
				swapped = true;
			}
			--i;
			if (i != first)
				--other;
		}
		if (!swapped)
			return;
		++flag;
		if (flag == last)
			return;
		first = flag;
		i = first; other = first;
		++other;
		swapped = false;
		while (i != last) {
			if (comp(*other, *i)) {
				swapit(*i, *other);
				flag = other;
				swapped = true;
			}
			++i;
			if (i != last)
				++other;
		}
		if (!swapped)
			return;
		last = flag;
		--last;
	} while (first != last);
}

/* In-place cyclic permutation of a container (no copying, only swapping). */
template <class It, class Swap>
void cyclic_permutation(It first, It last, It new_first, Swap swapit)
{
	unsigned num = last - first;
again:
	if (first == new_first || num < 2)
		return;

	unsigned num1 = new_first - first, num2 = last - new_first;
	if (num1 >= num2) {
		It a = first, b = new_first;
		while (b != last) {
			swapit(*a, *b);
			++a; ++b;
		}
		if (num1 > num2) {
			first += num2;
			num = num1;
			goto again;
		}
	} else {
		It a = new_first, b = last;
		do {
			--a; --b;
			swapit(*a, *b);
		} while (a != first);
		last -= num1;
		num = num2;
		goto again;
	}
}

/** Base class for generating all bounded combinatorial partitions of an integer
 *  n with exactly m parts in non-decreasing order.
 */
class basic_partition_generator {
protected:
	// Partitions n into m parts, not including zero parts.
	// (Cf. OEIS sequence A008284; implementation adapted from Jörg Arndt's
	// FXT library)
	struct mpartition2
	{
		// partition: x[1] + x[2] + ... + x[m] = n and sentinel x[0] == 0
		std::vector<unsigned> x;
		unsigned n;   // n>0
		unsigned m;   // 0<m<=n
		mpartition2(unsigned n_, unsigned m_)
		  : x(m_+1), n(n_), m(m_)
		{
			for (unsigned k=1; k<m; ++k)
				x[k] = 1;
			x[m] = n - m + 1;
		}
		bool next_partition()
		{
			unsigned u = x[m];  // last element
			unsigned k = m;
			unsigned s = u;
			while (--k) {
				s += x[k];
				if (x[k] + 2 <= u)
					break;
			}
			if (k==0)
				return false;  // current is last
			unsigned f = x[k] + 1;
			while (k < m) {
				x[k] = f;
				s -= f;
				++k;
			}
			x[m] = s;
			return true;
		}
	};
	mpartition2 mpgen;
	basic_partition_generator(unsigned n_, unsigned m_)
	  : mpgen(n_, m_)
	{ }
};

/** Generate all bounded combinatorial partitions of an integer n with exactly
 *  m parts (including zero parts) in non-decreasing order.
 */
class partition_with_zero_parts_generator : public basic_partition_generator {
private:
	unsigned m;  // number of parts 0<m
	mutable std::vector<unsigned> partition;  // current partition
	mutable bool current_updated;  // whether partition vector has been updated
public:
	partition_with_zero_parts_generator(unsigned n_, unsigned m_)
	  : basic_partition_generator(n_, 1), m(m_), partition(m_), current_updated(false)
	{ }
	// returns current partition in non-decreasing order, padded with zeros
	const std::vector<unsigned>& get() const
	{
		if (!current_updated) {
			for (unsigned i = 0; i < m - mpgen.m; ++i)
				partition[i] = 0;  // pad with zeros

			for (unsigned i = m - mpgen.m; i < m; ++i)
				partition[i] = mpgen.x[i - m + mpgen.m + 1];

			current_updated = true;
		}
		return partition;
	}
	bool next()
	{
		current_updated = false;
		if (!mpgen.next_partition()) {
			if (mpgen.m == m || mpgen.m == mpgen.n)
				return false;  // current is last
			// increment number of parts
			mpgen = mpartition2(mpgen.n, mpgen.m + 1);
		}
		return true;
	}
};

/** Generate all bounded combinatorial partitions of an integer n with exactly
 *  m parts (not including zero parts) in non-decreasing order.
 */
class partition_generator : public basic_partition_generator {
private:
	mutable std::vector<unsigned> partition;  // current partition
	mutable bool current_updated;  // whether partition vector has been updated
public:
	partition_generator(unsigned n_, unsigned m_)
	  : basic_partition_generator(n_, m_), partition(m_), current_updated(false)
	{ }
	// returns current partition in non-decreasing order, padded with zeros
	const std::vector<unsigned>& get() const
	{
		if (!current_updated) {
			for (unsigned i = 0; i < mpgen.m; ++i)
				partition[i] = mpgen.x[i + 1];

			current_updated = true;
		}
		return partition;
	}
	bool next()
	{
		current_updated = false;
		return mpgen.next_partition();
	}
};

/** Generate all compositions of a partition of an integer n, starting with the
 *  compositions which has non-decreasing order.
 */
class composition_generator {
private:
	// Generates all distinct permutations of a multiset.
	// (Based on Aaron Williams' algorithm 1 from "Loopless Generation of
	// Multiset Permutations using a Constant Number of Variables by Prefix
	// Shifts." <https://dl.acm.org/doi/pdf/10.5555/1496770.1496877>)
	struct coolmulti {
		// element of singly linked list
		struct element {
			unsigned value;
			element* next;
			element(unsigned val, element* n)
			  : value(val), next(n) {}
			~element()
			{   // recurses down to the end of the singly linked list
				delete next;
			}
		};
		element *head, *i, *after_i;
		// NB: Partition must be sorted in non-decreasing order.
		explicit coolmulti(const std::vector<unsigned>& partition)
		  : head(nullptr), i(nullptr), after_i(nullptr)
		{
			for (unsigned n = 0; n < partition.size(); ++n) {
				head = new element(partition[n], head);
				if (n <= 1)
					i = head;
			}
			after_i = i->next;
		}
		~coolmulti()
		{   // deletes singly linked list
			delete head;
		}
		void next_permutation()
		{
			element *before_k;
			if (after_i->next != nullptr && i->value >= after_i->next->value)
				before_k = after_i;
			else
				before_k = i;
			element *k = before_k->next;
			before_k->next = k->next;
			k->next = head;
			if (k->value < head->value)
				i = k;
			after_i = i->next;
			head = k;
		}
		bool finished() const
		{
			return after_i->next == nullptr && after_i->value >= head->value;
		}
	} cmgen;
	bool atend;  // needed for simplifying iteration over permutations
	bool trivial;  // likewise, true if all elements are equal
	mutable std::vector<unsigned> composition;  // current compositions
	mutable bool current_updated;  // whether composition vector has been updated
public:
	explicit composition_generator(const std::vector<unsigned>& partition)
	  : cmgen(partition), atend(false), trivial(true), composition(partition.size()), current_updated(false)
	{
		for (unsigned i=1; i<partition.size(); ++i)
			trivial = trivial && (partition[0] == partition[i]);
	}
	const std::vector<unsigned>& get() const
	{
		if (!current_updated) {
			coolmulti::element* it = cmgen.head;
			size_t i = 0;
			while (it != nullptr) {
				composition[i] = it->value;
				it = it->next;
				++i;
			}
			current_updated = true;
		}
		return composition;
	}
	bool next()
	{
		// This ugly contortion is needed because the original coolmulti
		// algorithm requires code duplication of the payload procedure,
		// one before the loop and one inside it.
		if (trivial || atend)
			return false;
		cmgen.next_permutation();
		current_updated = false;
		atend = cmgen.finished();
		return true;
	}
};

/** Compute the multinomial coefficient n!/(p1!*p2!*...*pk!) where
 *  n = p1+p2+...+pk, i.e. p is a partition of n.
 */
const numeric
multinomial_coefficient(const std::vector<unsigned> & p);


// Collection of `construct on first use' wrappers for safely avoiding
// internal object replication without running into the `static
// initialization order fiasco'.  This chest of numbers helps speed up
// the library but should not be used outside it since it is
// potentially confusing.

class ex;

extern const numeric *_num_120_p;
extern const ex _ex_120;
extern const numeric *_num_60_p;
extern const ex _ex_60;
extern const numeric *_num_48_p;
extern const ex _ex_48;
extern const numeric *_num_30_p;
extern const ex _ex_30;
extern const numeric *_num_25_p;
extern const ex _ex_25;
extern const numeric *_num_24_p;
extern const ex _ex_24;
extern const numeric *_num_20_p;
extern const ex _ex_20;
extern const numeric *_num_18_p;
extern const ex _ex_18;
extern const numeric *_num_15_p;
extern const ex _ex_15;
extern const numeric *_num_12_p;
extern const ex _ex_12;
extern const numeric *_num_11_p;
extern const ex _ex_11;
extern const numeric *_num_10_p;
extern const ex _ex_10;
extern const numeric *_num_9_p;
extern const ex _ex_9;
extern const numeric *_num_8_p;
extern const ex _ex_8;
extern const numeric *_num_7_p;
extern const ex _ex_7;
extern const numeric *_num_6_p;
extern const ex _ex_6;
extern const numeric *_num_5_p;
extern const ex _ex_5;
extern const numeric *_num_4_p;
extern const ex _ex_4;
extern const numeric *_num_3_p;
extern const ex _ex_3;
extern const numeric *_num_2_p;
extern const ex _ex_2;
extern const numeric *_num_1_p;
extern const ex _ex_1;
extern const numeric *_num_1_2_p;
extern const ex _ex_1_2;
extern const numeric *_num_1_3_p;
extern const ex _ex_1_3;
extern const numeric *_num_1_4_p;
extern const ex _ex_1_4;
extern const numeric *_num0_p;
extern const basic *_num0_bp;
extern const ex _ex0;
extern const numeric *_num1_4_p;
extern const ex _ex1_4;
extern const numeric *_num1_3_p;
extern const ex _ex1_3;
extern const numeric *_num1_2_p;
extern const ex _ex1_2;
extern const numeric *_num1_p;
extern const ex _ex1;
extern const numeric *_num2_p;
extern const ex _ex2;
extern const numeric *_num3_p;
extern const ex _ex3;
extern const numeric *_num4_p;
extern const ex _ex4;
extern const numeric *_num5_p;
extern const ex _ex5;
extern const numeric *_num6_p;
extern const ex _ex6;
extern const numeric *_num7_p;
extern const ex _ex7;
extern const numeric *_num8_p;
extern const ex _ex8;
extern const numeric *_num9_p;
extern const ex _ex9;
extern const numeric *_num10_p;
extern const ex _ex10;
extern const numeric *_num11_p;
extern const ex _ex11;
extern const numeric *_num12_p;
extern const ex _ex12;
extern const numeric *_num15_p;
extern const ex _ex15;
extern const numeric *_num18_p;
extern const ex _ex18;
extern const numeric *_num20_p;
extern const ex _ex20;
extern const numeric *_num24_p;
extern const ex _ex24;
extern const numeric *_num25_p;
extern const ex _ex25;
extern const numeric *_num30_p;
extern const ex _ex30;
extern const numeric *_num48_p;
extern const ex _ex48;
extern const numeric *_num60_p;
extern const ex _ex60;
extern const numeric *_num120_p;
extern const ex _ex120;


// Helper macros for class implementations (mostly useful for trivial classes)

#define DEFAULT_CTOR(classname) \
classname::classname() { setflag(status_flags::evaluated | status_flags::expanded); }

#define DEFAULT_COMPARE(classname) \
int classname::compare_same_type(const basic & other) const \
{ \
	/* by default, the objects are always identical */ \
	return 0; \
}

#define DEFAULT_PRINT(classname, text) \
void classname::do_print(const print_context & c, unsigned level) const \
{ \
	c.s << text; \
}

#define DEFAULT_PRINT_LATEX(classname, text, latex) \
DEFAULT_PRINT(classname, text) \
void classname::do_print_latex(const print_latex & c, unsigned level) const \
{ \
	c.s << latex; \
}

} // namespace GiNaC

#endif // ndef GINAC_UTILS_H
