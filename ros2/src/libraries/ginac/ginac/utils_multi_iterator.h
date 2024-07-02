/** @file utils_multi_iterator.h
 *
 *  Utilities for summing over multiple indices */

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

#ifndef GINAC_UTILS_MULTI_ITERATOR_H
#define GINAC_UTILS_MULTI_ITERATOR_H

#include <cstddef>
#include <vector>
#include <ostream>
#include <iterator>

namespace GiNaC {

/**
 *
 * SFINAE test for distance
 *
 */
template <typename T> class has_distance {
private:
	typedef char yes_type[1];
	typedef char no_type[2];

	template <typename C> static yes_type & test( decltype(std::distance<C>) ) ;
	template <typename C> static no_type & test(...);

public:
	enum { value = sizeof(test<T>(0)) == sizeof(yes_type) };
};

/**
 *
 * For printing a multi-index: 
 * If the templates are used, where T is an iterator, printing the address where the iterator points to is not meaningful.
 * However, we may print the difference to the starting point.
 *
 */
template<typename T> typename std::enable_if<has_distance<T>::value, typename std::iterator_traits<T>::difference_type>::type format_index_value(const T & a, const T & b) {
		return std::distance(a,b);
}

/**
 *
 * For all other cases we simply print the value.
 *
 */
template<typename T> typename std::enable_if<!has_distance<T>::value, T>::type format_index_value(const T & a, const T & b) {
	return b;
}

/**
 *
 * basic_multi_iterator is a base class.
 *
 * The base class itself does not do anything useful.
 * A typical use of a class derived from basic_multi_iterator is
 *
 *    multi_iterator_ordered<int> k(0,4,2);
 *
 *    for( k.init(); !k.overflow(); k++) {
 *	std::cout << k << std::endl;
 *    }
 *
 * which prints out 
 *
 *   multi_iterator_ordered(0,1)
 *   multi_iterator_ordered(0,2)
 *   multi_iterator_ordered(0,3)
 *   multi_iterator_ordered(1,2)
 *   multi_iterator_ordered(1,3)
 *   multi_iterator_ordered(2,3)
 *
 * Individual components of k can be accessed with k[i] or k(i).
 *
 * All classes derived from basic_multi_iterator follow the same syntax.
 *
 */
template<class T> class basic_multi_iterator {

	// ctors
public :  
	basic_multi_iterator(void);
	explicit basic_multi_iterator(T B, T N, size_t k);
	explicit basic_multi_iterator(T B, T N, const std::vector<T> & vv);

	// dtor 
	virtual ~basic_multi_iterator();
 
	// functions 
public :
	size_t size(void) const;
	bool overflow(void) const;
	const std::vector<T> & get_vector(void) const;

	// subscripting
public :  
	T operator[](size_t i) const;
	T & operator[](size_t i);

	T operator()(size_t i) const;
	T & operator()(size_t i);

	// virtual functions
public :  
	// initialization
	virtual basic_multi_iterator<T> & init(void);
	// postfix increment
	virtual basic_multi_iterator<T> & operator++ (int);
 
	// I/O operators
	template <class TT> friend std::ostream & operator<< (std::ostream & os, const basic_multi_iterator<TT> & v);

	// member variables :
protected : 
	T N;
	T B;
	std::vector<T> v;
	bool flag_overflow;

};

/**
 *
 * The class multi_iterator_ordered defines a multi_iterator
 * \f$(i_1,i_2,...,i_k)\f$, such that
 * \f[
 *     B \le i_j < N
 * \f]
 * and
 * \f[
 *     i_j < i_{j+1}.
 * \f]
 * It is assumed that \f$k>0\f$ and \f$ N-B \ge k \f$.
 * 
 */
template<class T> class multi_iterator_ordered : public basic_multi_iterator<T> {

	// ctors
public :  
	multi_iterator_ordered(void);
	explicit multi_iterator_ordered(T B, T N, size_t k);
	explicit multi_iterator_ordered(T B, T N, const std::vector<T> & vv);

	// overriding virtual functions from base class
public :  
	// initialization
	basic_multi_iterator<T> & init(void);
	// postfix increment
	basic_multi_iterator<T> & operator++ (int);
 
	// I/O operators
	template <class TT> friend std::ostream & operator<< (std::ostream & os, const multi_iterator_ordered<TT> & v);

};

/**
 *
 * The class multi_iterator_ordered_eq defines a multi_iterator
 * \f$(i_1,i_2,...,i_k)\f$, such that
 * \f[
 *     B \le i_j < N
 * \f]
 * and
 * \f[
 *     i_j \le i_{j+1}.
 * \f]
 * It is assumed that \f$k>0\f$.
 * 
 */
template<class T> class multi_iterator_ordered_eq : public basic_multi_iterator<T> {

	// ctors
public :  
	multi_iterator_ordered_eq(void);
	explicit multi_iterator_ordered_eq(T B, T N, size_t k);
	explicit multi_iterator_ordered_eq(T B, T N, const std::vector<T> & vv);

	// overriding virtual functions from base class
public :  
	// initialization
	basic_multi_iterator<T> & init(void);
	// postfix increment
	basic_multi_iterator<T> & operator++ (int);
 
	// I/O operators
	template <class TT> friend std::ostream & operator<< (std::ostream & os, const multi_iterator_ordered_eq<TT> & v);

};

/**
 *
 * The class multi_iterator_ordered_eq_indv defines a multi_iterator
 * \f$(i_1,i_2,...,i_k)\f$, such that
 * \f[
 *     B \le i_j < N_j
 * \f]
 * and
 * \f[
 *     i_j \le i_{j+1}.
 * \f]
 * 
 */
template<class T> class multi_iterator_ordered_eq_indv : public basic_multi_iterator<T> {

	// ctors
public :  
	multi_iterator_ordered_eq_indv(void);
	explicit multi_iterator_ordered_eq_indv(T B, const std::vector<T> & Nv, size_t k);
	explicit multi_iterator_ordered_eq_indv(T B, const std::vector<T> & Nv, const std::vector<T> & vv);

	// overriding virtual functions from base class
public :  
	// initialization
	basic_multi_iterator<T> & init(void);
	// postfix increment
	basic_multi_iterator<T> & operator++ (int);
 
	// I/O operators
	template <class TT> friend std::ostream & operator<< (std::ostream & os, const multi_iterator_ordered_eq_indv<TT> & v);

	// member variables :
protected : 
	std::vector<T> Nv;
};

/**
 *
 * The class multi_iterator_counter defines a multi_iterator
 * \f$(i_1,i_2,...,i_k)\f$, such that
 * \f[
 *     B \le i_j < N
 * \f]
 * 
 */
template<class T> class multi_iterator_counter : public basic_multi_iterator<T> {

	// ctors
public :  
	multi_iterator_counter(void);
	explicit multi_iterator_counter(T B, T N, size_t k);
	explicit multi_iterator_counter(T B, T N, const std::vector<T> & vv);

	// overriding virtual functions from base class
public :  
	// initialization
	basic_multi_iterator<T> & init(void);
	// postfix increment
	basic_multi_iterator<T> & operator++ (int);
 
	// I/O operators
	template <class TT> friend std::ostream & operator<< (std::ostream & os, const multi_iterator_counter<TT> & v);

};

/**
 *
 * The class multi_iterator_counter_indv defines a multi_iterator
 * \f$(i_1,i_2,...,i_k)\f$, such that
 * \f[
 *     B \le i_j < N_j
 * \f]
 * 
 */
template<class T> class multi_iterator_counter_indv : public basic_multi_iterator<T> {

	// ctors
public :  
	multi_iterator_counter_indv(void);
	explicit multi_iterator_counter_indv(T B, const std::vector<T> & Nv, size_t k);
	explicit multi_iterator_counter_indv(T B, const std::vector<T> & Nv, const std::vector<T> & vv);

	// overriding virtual functions from base class
public :  
	// initialization
	basic_multi_iterator<T> & init(void);
	// postfix increment
	basic_multi_iterator<T> & operator++ (int);
 
	// I/O operators
	template <class TT> friend std::ostream & operator<< (std::ostream & os, const multi_iterator_counter_indv<TT> & v);

	// member variables :
protected : 
	std::vector<T> Nv;
};

/**
 *
 * The class multi_iterator_permutation defines a multi_iterator
 * \f$(i_1,i_2,...,i_k)\f$, for which
 * \f[
 *    B \le i_j < N
 * \f]
 * and
 * \f[
 *     i_i \neq i_j
 * \f]
 * In particular, if \f$N-B=k\f$, multi_iterator_permutation loops over all
 * permutations of \f$k\f$ elements.
 * 
 */
template<class T> class multi_iterator_permutation : public basic_multi_iterator<T> {

	// ctors
public :  
	multi_iterator_permutation(void);
	explicit multi_iterator_permutation(T B, T N, size_t k);
	explicit multi_iterator_permutation(T B, T N, const std::vector<T> & vv);

	// overriding virtual functions from base class
public :  
	// initialization
	basic_multi_iterator<T> & init(void);
	// postfix increment
	basic_multi_iterator<T> & operator++ (int);

	// new functions in this class
	int get_sign(void) const;
 
	// I/O operators
	template <class TT> friend std::ostream & operator<< (std::ostream & os, const multi_iterator_permutation<TT> & v);

};

/**
 *
 * The class multi_iterator_shuffle defines a multi_iterator,
 * which runs over all shuffles of a and b.
 * 
 */
template<class T> class multi_iterator_shuffle : public basic_multi_iterator<T> {

	// ctors
public :  
	multi_iterator_shuffle(void);
	explicit multi_iterator_shuffle(const std::vector<T> & a, const std::vector<T> & b);

	// overriding virtual functions from base class
public :  
	// initialization
	basic_multi_iterator<T> & init(void);
	// postfix increment
	basic_multi_iterator<T> & operator++ (int);
 
	// I/O operators
	template <class TT> friend std::ostream & operator<< (std::ostream & os, const multi_iterator_shuffle<TT> & v);

	// member variables :
protected : 
	size_t N_internal;
	std::vector<size_t> v_internal;
	std::vector<T> v_orig;
};

/**
 *
 * The class multi_iterator_shuffle_prime defines a multi_iterator,
 * which runs over all shuffles of a and b, excluding the first one (a,b).
 * 
 */
template<class T> class multi_iterator_shuffle_prime : public multi_iterator_shuffle<T> {

	// ctors
public :  
	multi_iterator_shuffle_prime(void);
	explicit multi_iterator_shuffle_prime(const std::vector<T> & a, const std::vector<T> & b);

	// overriding virtual functions from base class
public :  
	// initialization
	basic_multi_iterator<T> & init(void);
 
	// I/O operators
	template <class TT> friend std::ostream & operator<< (std::ostream & os, const multi_iterator_shuffle_prime<TT> & v);
};

// ----------------------------------------------------------------------------------------------------------------

// ctors

/**
 *
 * Default constructor
 *
 */
template<class T> inline basic_multi_iterator<T>::basic_multi_iterator(void) : N(), B(), v(), flag_overflow(false)
{}

/**
 *
 * Construct a multi_iterator with upper limit N, lower limit B and size k .
 *
 */
template<class T> inline basic_multi_iterator<T>::basic_multi_iterator(T BB, T NN, size_t k) : N(NN), B(BB), v(k), flag_overflow(false)
{}

/**
 *
 * Construct from a vector.
 *
 */
template<class T> inline basic_multi_iterator<T>::basic_multi_iterator(T BB, T NN, const std::vector<T> & vv) : N(NN), B(BB), v(vv), flag_overflow(false)
{}

/**
 *
 * Destructor
 *
 */
template<class T> inline basic_multi_iterator<T>::~basic_multi_iterator()
{}

// functions 

/**
 *
 * Returns the size of a multi_iterator.
 *
 */
template<class T> inline size_t basic_multi_iterator<T>::size(void) const
{
	return v.size();
}

/**
 *
 * Initialize the multi-index to
 * \f[
 *    (n_1,n_2,n_3,...,n_k) = (B,B,...,B)
 * \f]
 *
 */
template<class T> inline basic_multi_iterator<T> & basic_multi_iterator<T>::init(void) 
{
	flag_overflow = false;

	for ( size_t i=0; i<v.size(); i++) {
		v[i] = B;
	}
	return *this;
}

/**
 *
 * Return the overflow flag.
 *
 */
template<class T> inline bool basic_multi_iterator<T>::overflow(void) const
{
	return flag_overflow;
}

/**
 *
 * Returns a reference to the vector v.
 *
 */
template<class T> inline const std::vector<T> & basic_multi_iterator<T>::get_vector(void) const
{
	return v;
}

// subscripting

/**
 *
 * Subscription via []
 *
 */
template<class T> inline T basic_multi_iterator<T>::operator[](size_t i) const
{
	return v[i];
}

/**
 *
 * Subscription via []
 *
 */
template<class T> inline T & basic_multi_iterator<T>::operator[](size_t i)
{
	return v[i];
}

/**
 *
 * Subscription via ()
 *
 */
template<class T> inline T basic_multi_iterator<T>::operator()(size_t i) const
{
	return v[i];
}

/**
 *
 * Subscription via ()
 *
 */
template<class T> inline T & basic_multi_iterator<T>::operator()(size_t i)
{
	return v[i];
}


/**
 *
 * No effect for basic_multi_iterator
 *
 */
template<class T> inline basic_multi_iterator<T> & basic_multi_iterator<T>::operator++ (int)
{
	return *this;
}

// I/O operators

/**
 *
 * Output operator. A multi_iterator prints out as 
 * basic_multi_iterator(\f$n_0,n_1,...\f$).
 *
 */
template<class T> inline std::ostream & operator<< (std::ostream & os, const basic_multi_iterator<T> & v)
{
	os << "basic_multi_iterator(";
	for ( size_t i=0; i<v.size(); i++) {
		if (i>0) {
			os << ",";
		}
		os << format_index_value(v.B,v(i));
	}
	
	return os << ")";
}



// ctors

/**
 *
 * Default constructor
 *
 */
template<class T> inline multi_iterator_ordered<T>::multi_iterator_ordered(void) : basic_multi_iterator<T>()
{}

/**
 *
 * Construct a multi_iterator with upper limit N and size k .
 *
 */
template<class T> inline multi_iterator_ordered<T>::multi_iterator_ordered(T B, T N, size_t k) : basic_multi_iterator<T>(B,N,k)
{}

/**
 *
 * Construct from a vector.
 *
 */
template<class T> inline multi_iterator_ordered<T>::multi_iterator_ordered(T B, T N, const std::vector<T> & v) : basic_multi_iterator<T>(B,N,v)
{}

// functions 

/**
 *
 * Initialize the multi-index to
 * \f[
 *    (n_1,n_2,n_3,...,n_k) = (B+0,B+1,B+2,...,B+k-1)
 * \f]
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_ordered<T>::init(void) 
{
	this->flag_overflow = false;
	T it = this->B;

	for ( size_t i=0; i < this->v.size(); i++) {
		this->v[i] = it;
		it++;
	}
	return *this;
}

/**
 *
 * The postfix increment operator allows to
 * write for a multi-index n++, which will
 * update n to the next configuration.
 *
 * If n is in the last configuration and the
 * increment operator ++ is applied to n,
 * the overflow flag will be raised.
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_ordered<T>::operator++ (int)
{
	int k = this->size();
	int j = k - 1;
	T Upper_limit = this->N;

	while ( j>0 ) {
		this->v[j]++;
		if ( this->v[j] == Upper_limit ) {
			j--;
			Upper_limit--;
		}
		else {
			break;
		}
	}

	if (j==0) {
		this->v[j]++;
		if (this->v[j] == Upper_limit) this->flag_overflow=true;
	}

	if ( j>= 0) {
		for (int jj=j+1;jj<k;jj++) {
			this->v[jj] = this->v[jj-1];
			this->v[jj]++;
		}
	}

	return *this;
}

// I/O operators

/**
 *
 * Output operator. A multi_iterator_ordered prints out as 
 * multi_iterator_ordered(\f$n_0,n_1,...\f$).
 *
 */
template<class T> inline std::ostream & operator<< (std::ostream & os, const multi_iterator_ordered<T> & v)
{
	os << "multi_iterator_ordered(";
	for ( size_t i=0; i<v.size(); i++) {
		if (i>0) {
			os << ",";
		}
		os << format_index_value(v.B,v(i));
	}

	return os << ")";
}



// ctors

/**
 *
 * Default constructor
 *
 */
template<class T> inline multi_iterator_ordered_eq<T>::multi_iterator_ordered_eq(void) : basic_multi_iterator<T>()
{}

/**
 *
 * Construct a multi_iterator with upper limit N and size k .
 *
 */
template<class T> inline multi_iterator_ordered_eq<T>::multi_iterator_ordered_eq(T B, T N, size_t k) : basic_multi_iterator<T>(B,N,k)
{}

/**
 *
 * Construct from a vector.
 *
 */
template<class T> inline multi_iterator_ordered_eq<T>::multi_iterator_ordered_eq(T B, T N, const std::vector<T> & v) : basic_multi_iterator<T>(B,N,v)
{}

// functions 

/**
 *
 * Initialize the multi-index to
 * \f[
 *    (n_1,n_2,...,n_k) = (B,B,...,B)
 * \f]
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_ordered_eq<T>::init(void) 
{
	this->flag_overflow = false;

	for ( size_t i=0; i < this->v.size(); i++) {
		this->v[i] = this->B;
	}
	return *this;
}

/**
 *
 * The postfix increment operator allows to
 * write for a multi-index n++, which will
 * update n to the next configuration.
 *
 * If n is in the last configuration and the
 * increment operator ++ is applied to n,
 * the overflow flag will be raised.
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_ordered_eq<T>::operator++ (int)
{
	int k = this->size();
	int j = k - 1;

	while ( j>0 ) {
		this->v[j]++;
		if ( this->v[j] == this->N ) {
			j--;
		}
		else {
			break;
		}
	}

	if (j==0) {
		this->v[j]++;
		if (this->v[j] == this->N) {
			this->flag_overflow=true;
		}
	}

	if ( j>= 0) {
		for (int jj=j+1;jj<k;jj++) {
			this->v[jj] = this->v[jj-1];
		}
	}

	return *this;
}

// I/O operators

/**
 *
 * Output operator. A multi_iterator_ordered_eq prints out as 
 * multi_iterator_ordered_eq(\f$n_0,n_1,...\f$).
 *
 */
template<class T> inline std::ostream & operator<< (std::ostream & os, const multi_iterator_ordered_eq<T> & v)
{
	os << "multi_iterator_ordered_eq(";
	for ( size_t i=0; i<v.size(); i++) {
		if (i>0) {
			os << ",";
		}
		os << format_index_value(v.B,v(i));
	}

	return os << ")";
}




// ctors

/**
 *
 * Default constructor
 *
 */
template<class T> inline multi_iterator_ordered_eq_indv<T>::multi_iterator_ordered_eq_indv(void) : basic_multi_iterator<T>(), Nv()
{}

/**
 *
 * Construct a multi_iterator with upper limit N and size k .
 *
 */
template<class T> inline multi_iterator_ordered_eq_indv<T>::multi_iterator_ordered_eq_indv(T B, const std::vector<T> & Nvv, size_t k) : basic_multi_iterator<T>(B,B,k), Nv(Nvv)
{}

/**
 *
 * Construct from a vector.
 *
 */
template<class T> inline multi_iterator_ordered_eq_indv<T>::multi_iterator_ordered_eq_indv(T B, const std::vector<T> & Nvv, const std::vector<T> & v) : basic_multi_iterator<T>(B,B,v), Nv(Nvv)
{}

// functions 

/**
 *
 * Initialize the multi-index to
 * \f[
 *    (n_1,n_2,n_3,...,n_k) = (B,B,B,...,B)
 * \f]
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_ordered_eq_indv<T>::init(void) 
{
	this->flag_overflow = false;

	for ( size_t i=0; i < this->v.size(); i++) {
		this->v[i] = this->B;
	}
	return *this;
}

/**
 *
 * The postfix increment operator allows to
 * write for a multi-index n++, which will
 * update n to the next configuration.
 *
 * If n is in the last configuration and the
 * increment operator ++ is applied to n,
 * the overflow flag will be raised.
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_ordered_eq_indv<T>::operator++ (int)
{
	int k = this->size();
	int j = k - 1;

	while ( j>0 ) {
		this->v[j]++;
		if ( this->v[j] == Nv[j] ) {
			j--;
		}
		else {
			break;
		}
	}

	if (j==0) {
		this->v[j]++;
		if (this->v[j] == Nv[j]) {
			this->flag_overflow=true;
		}
	}

	if ( j>= 0) {
		for (int jj=j+1;jj<k;jj++) {
			this->v[jj] = this->v[jj-1];
		}
	}

	return *this;
}

// I/O operators

/**
 *
 * Output operator. A multi_iterator_ordered_eq_indv prints out as 
 * multi_iterator_ordered_eq_indv(\f$n_0,n_1,...\f$).
 *
 */
template<class T> inline std::ostream & operator<< (std::ostream & os, const multi_iterator_ordered_eq_indv<T> & v)
{
	os << "multi_iterator_ordered_eq_indv(";
	for ( size_t i=0; i<v.size(); i++) {
		if (i>0) {
			os << ",";
		}
		os << format_index_value(v.B,v(i));
	}

	return os << ")";
}




// ctors

/**
 *
 * Default constructor
 *
 */
template<class T> inline multi_iterator_counter<T>::multi_iterator_counter(void) : basic_multi_iterator<T>()
{}

/**
 *
 * Construct a multi_iterator with upper limit N and size k .
 *
 */
template<class T> inline multi_iterator_counter<T>::multi_iterator_counter(T B, T N, size_t k) : basic_multi_iterator<T>(B,N,k)
{}

/**
 *
 * Construct from a vector.
 *
 */
template<class T> inline multi_iterator_counter<T>::multi_iterator_counter(T B, T N, const std::vector<T> & v) : basic_multi_iterator<T>(B,N,v)
{}

// functions 

/**
 *
 * Initialize the multi-index to
 * \f[
 *    (n_1,n_2,n_3,...,n_k) = (B,B,...,B)
 * \f]
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_counter<T>::init(void) 
{
	this->flag_overflow = false;

	for ( size_t i=0; i < this->v.size(); i++) {
		this->v[i] = this->B;
	}
	return *this;
}

/**
 *
 * The postfix increment operator allows to
 * write for a multi-index n++, which will
 * update n to the next configuration.
 *
 * If n is in the last configuration and the
 * increment operator ++ is applied to n,
 * the overflow flag will be raised.
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_counter<T>::operator++ (int)
{
	int k = this->size();
	int j = k - 1;

	while ( j>0 ) {
		this->v[j]++;
		if ( this->v[j] == this->N ) {
			this->v[j] = this->B;
			j--;
		}
		else {
			break;
		}
	}

	if (j==0) {
		this->v[j]++;
		if (this->v[j] == this->N) {
			this->v[j] = this->B;
			this->flag_overflow=true;
		}
	}

	return *this;
}

// I/O operators

/**
 *
 * Output operator. A multi_iterator_counter prints out as 
 * multi_iterator_counter(\f$n_0,n_1,...\f$).
 *
 */
template<class T> inline std::ostream & operator<< (std::ostream & os, const multi_iterator_counter<T> & v)
{
	os << "multi_iterator_counter(";
	for ( size_t i=0; i<v.size(); i++) {
		if (i>0) {
			os << ",";
		}
		os << format_index_value(v.B,v(i));
	}

	return os << ")";
}




// ctors

/**
 *
 * Default constructor
 *
 */
template<class T> inline multi_iterator_counter_indv<T>::multi_iterator_counter_indv(void) : basic_multi_iterator<T>(), Nv()
{}

/**
 *
 * Construct a multi_iterator with upper limit N and size k .
 *
 */
template<class T> inline multi_iterator_counter_indv<T>::multi_iterator_counter_indv(T B, const std::vector<T> & Nvv, size_t k) : basic_multi_iterator<T>(B,B,k), Nv(Nvv)
{}

/**
 *
 * Construct from a vector.
 *
 */
template<class T> inline multi_iterator_counter_indv<T>::multi_iterator_counter_indv(T B, const std::vector<T> & Nvv, const std::vector<T> & v) : basic_multi_iterator<T>(B,B,v), Nv(Nvv)
{}

// functions 

/**
 *
 * Initialize the multi-index to
 * \f[
 *    (n_1,n_2,n_3,...,n_k) = (B,B,...,B)
 * \f]
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_counter_indv<T>::init(void) 
{
	this->flag_overflow = false;

	for ( size_t i=0; i < this->v.size(); i++) {
		this->v[i] = this->B;
	}
	return *this;
}

/**
 *
 * The postfix increment operator allows to
 * write for a multi-index n++, which will
 * update n to the next configuration.
 *
 * If n is in the last configuration and the
 * increment operator ++ is applied to n,
 * the overflow flag will be raised.
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_counter_indv<T>::operator++ (int)
{
	int k = this->size();
	int j = k - 1;

	while ( j>0 ) {
		this->v[j]++;
		if ( this->v[j] == Nv[j] ) {
			this->v[j] = this->B;
			j--;
		}
		else {
			break;
		}
	}

	if (j==0) {
		this->v[j]++;
		if (this->v[j] == Nv[j]) {
			this->v[j] = this->B;
			this->flag_overflow=true;
		}
	}

	return *this;
}

// I/O operators

/**
 *
 * Output operator. A multi_iterator_counter_indv prints out as 
 * multi_iterator_counter_indv(\f$n_0,n_1,...\f$).
 *
 */
template<class T> inline std::ostream & operator<< (std::ostream & os, const multi_iterator_counter_indv<T> & v)
{
	os << "multi_iterator_counter_indv(";
	for ( size_t i=0; i<v.size(); i++) {
		if (i>0) {
			os << ",";
		}
		os << format_index_value(v.B,v(i));
	}

	return os << ")";
}




// ctors

/**
 *
 * Default constructor
 *
 */
template<class T> inline multi_iterator_permutation<T>::multi_iterator_permutation(void) : basic_multi_iterator<T>()
{}

/**
 *
 * Construct a multi_iterator with upper limit N and size k .
 *
 */
template<class T> inline multi_iterator_permutation<T>::multi_iterator_permutation(T B, T N, size_t k) : basic_multi_iterator<T>(B,N,k)
{}

/**
 *
 * Construct from a vector.
 *
 */
template<class T> inline multi_iterator_permutation<T>::multi_iterator_permutation(T B, T N, const std::vector<T> & v) : basic_multi_iterator<T>(B,N,v)
{}

// functions 

/**
 *
 * Initialize the multi-index to
 * \f[
 *    (n_1,n_2,n_3,...,n_k) = (B+0,B+1,B+2,...,B+k-1)
 * \f]
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_permutation<T>::init(void) 
{
	this->flag_overflow = false;
	T it = this->B;

	for ( size_t i=0; i < this->v.size(); i++) {
		this->v[i] = it;
		it++;
	}
	return *this;
}

/**
 *
 * The postfix increment operator allows to
 * write for a multi-index n++, which will
 * update n to the next configuration.
 *
 * If n is in the last configuration and the
 * increment operator ++ is applied to n,
 * the overflow flag will be raised.
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_permutation<T>::operator++ (int)
{
	int k = this->size();
	int j = k - 1;

	while ( j>=0 ) {
		bool flag_have_already = true;
		while ( flag_have_already ) {
			this->v[j]++;

			// update flag_have_already
			flag_have_already = false;
			for (int ii=0; ii<j; ii++) {
				if (this->v[j] == this->v[ii]) {
					flag_have_already = true;
				}
			}
		}

		if ( this->v[j] == this->N ) {
			j--;
		}
		else {
			break;
		}
	}

	for (int l=j+1; l<k; l++) {
		this->v[l] = this->B;

		bool flag_have_already;
		do {
			flag_have_already = false;
			for (int ii=0; ii<l; ii++) {
				if (this->v[l] == this->v[ii]) {
					flag_have_already = true;
				}
			}
			if (flag_have_already) {
				this->v[l]++;
			}
		}
		while (flag_have_already);
	}

	// check for overflow
	this->flag_overflow = true;
	T it = this->B;
	for (int ii=0; ii<k; ii++) {
		if (this->v[ii] != it) {
			this->flag_overflow = false;
		}
		it++;
	}

	return *this;
}

/**
 *
 * Returns the sign of the permutation, defined by
 * \f[
 *     \left(-1\right)^{n_{inv}},
 * \f]
 * where \f$ n_{inv} \f$ is the number of inversions, e.g. the
 * number of pairs \f$ i < j \f$ for which
 * \f[
 *     n_i > n_j.
 * \f]
 *     
 */
template<class T> inline int multi_iterator_permutation<T>::get_sign() const
{
	int sign = 1;
	int k = this->size();

	for ( int i=0; i<k; i++) {
		for ( int j=i+1; j<k; j++) {
			// works only for random-access iterators
			if ( this->v[i] > this->v[j] ) {
				sign = -sign;
			}
		}
	}

	return sign;
}


// I/O operators

/**
 *
 * Output operator. A multi_iterator_permutation prints out as 
 * multi_iterator_permutation(\f$n_0,n_1,...\f$).
 *
 */
template<class T> inline std::ostream & operator<< (std::ostream & os, const multi_iterator_permutation<T> & v)
{
	os << "multi_iterator_permutation(";
	for ( size_t i=0; i<v.size(); i++) {
		if (i>0) {
			os << ",";
		}
		os << format_index_value(v.B,v(i));
	}

	return os << ")";
}



// ctors

/**
 *
 * Default constructor
 *
 */
template<class T> inline multi_iterator_shuffle<T>::multi_iterator_shuffle(void) : basic_multi_iterator<T>(), N_internal(), v_internal(), v_orig()
{}

/**
 *
 * Construct from a vector.
 *
 */
template<class T> inline multi_iterator_shuffle<T>::multi_iterator_shuffle(const std::vector<T> & a, const std::vector<T> & b) : basic_multi_iterator<T>(), N_internal(), v_internal(), v_orig()
{
	this->B = a[0];

	for (size_t i=0; i<a.size(); i++) {
		this->v.push_back( a[i] );
		this->v_orig.push_back( a[i] );
		this->v_internal.push_back( i );
	}
	for (size_t i=0; i<b.size(); i++) {
		this->v.push_back( b[i] );
		this->v_orig.push_back( b[i] );
	}
	this->N_internal = this->v.size();
}

// functions 

/**
 *
 * Initialize the multi-index to the first shuffle.
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_shuffle<T>::init(void) 
{
	this->flag_overflow = false;

	for ( size_t i=0; i < this->v_internal.size(); i++) {
		this->v_internal[i] = i;
	}
	for ( size_t i=0; i < this->v.size(); i++) {
		this->v[i] = this->v_orig[i];
	}
	return *this;
}

/**
 *
 * The postfix increment operator allows to
 * write for a multi-index n++, which will
 * update n to the next configuration.
 *
 * If n is in the last configuration and the
 * increment operator ++ is applied to n,
 * the overflow flag will be raised.
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_shuffle<T>::operator++ (int)
{
	int k = this->v_internal.size();
	int j = k - 1;
	size_t Upper_limit = this->N_internal;

	while ( j>0 ) {
		this->v_internal[j]++;
		if ( this->v_internal[j] == Upper_limit ) {
			j--;
			Upper_limit--;
		}
		else {
			break;
		}
	}

	if (j==0) {
		this->v_internal[j]++;
		if (this->v_internal[j] == Upper_limit) {
			this->flag_overflow=true;
		}
	}

	if ( j>= 0) {
		for (int jj=j+1;jj<k;jj++) {
			this->v_internal[jj] = this->v_internal[jj-1];
			this->v_internal[jj]++;
		}
	}

	// update v
	if ( !(this->flag_overflow) ) {
		size_t i_a = 0;
		size_t i_b = 0;
		size_t i_all = 0;
		for (size_t j=0; j<k; j++) {
			for (size_t i=i_all; i < this->v_internal[j]; i++) {
				this->v[i_all] = this->v_orig[k+i_b];
				i_b++;
				i_all++;
			}
			this->v[i_all] = this->v_orig[i_a];
			i_a++;
			i_all++;
		}
		for (size_t i = this->v_internal[k-1]+1; i < this->v.size(); i++) {
			this->v[i_all] = this->v_orig[k+i_b];
			i_b++;
			i_all++;
		}
	}

	return *this;
}

// I/O operators

/**
 *
 * Output operator. A multi_iterator_shuffle prints out as 
 * multi_iterator_shuffle(\f$n_0,n_1,...\f$).
 *
 */
template<class T> inline std::ostream & operator<< (std::ostream & os, const multi_iterator_shuffle<T> & v)
{
	os << "multi_iterator_shuffle(";
	for ( size_t i=0; i<v.size(); i++) {
		if (i>0) {
			os << ",";
		}
		os << format_index_value(v.B,v(i));
	}

	return os << ")";
}



// ctors

/**
 *
 * Default constructor
 *
 */
template<class T> inline multi_iterator_shuffle_prime<T>::multi_iterator_shuffle_prime(void) : multi_iterator_shuffle<T>()
{}

/**
 *
 * Construct from a vector.
 *
 */
template<class T> inline multi_iterator_shuffle_prime<T>::multi_iterator_shuffle_prime(const std::vector<T> & a, const std::vector<T> & b) : multi_iterator_shuffle<T>(a,b)
{}

// functions 

/**
 *
 * Initialize the multi-index to the first shuffle.
 *
 */
template<class T> inline basic_multi_iterator<T> & multi_iterator_shuffle_prime<T>::init(void) 
{
	this->flag_overflow = false;

	for ( size_t i=0; i < this->v_internal.size(); i++) {
		this->v_internal[i] = i;
	}
	for ( size_t i=0; i < this->v.size(); i++) {
		this->v[i] = this->v_orig[i];
	}

	(*this)++;

	return *this;
}

// I/O operators

/**
 *
 * Output operator. A multi_iterator_shuffle_prime prints out as 
 * multi_iterator_shuffle_prime(\f$n_0,n_1,...\f$).
 *
 */
template<class T> inline std::ostream & operator<< (std::ostream & os, const multi_iterator_shuffle_prime<T> & v)
{
	os << "multi_iterator_shuffle_prime(";
	for ( size_t i=0; i<v.size(); i++) {
		if (i>0) {
			os << ",";
		}
		os << format_index_value(v.B,v(i));
	}

	return os << ")";
}

} // namespace GiNaC

#endif // ndef GINAC_UTILS_MULTI_ITERATOR_H
