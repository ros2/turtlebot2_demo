/**
 * @file /ecl_sigslots/include/ecl/sigslots/sigslot.hpp
 *
 * @brief File comment
 *
 * File comment
 *
 * @date 13/05/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SIGSLOTS_SIGSLOT_HPP_
#define ECL_SIGSLOTS_SIGSLOT_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <map>
#include <set>
#include <string>
#include <ecl/config/macros.hpp>
#include <ecl/threads/mutex.hpp>
#include <ecl/utilities/function_objects.hpp>
#include <ecl/utilities/void.hpp>
#include "manager.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface [General]
*****************************************************************************/
/**
 * @brief Not for direct use, provides the power behind both signals and slots.
 *
 * This is the workhorse for both signals and slots, providing the implementation
 * for all the functions necessary by both types of frontends.
 */
template <typename Data=Void>
class SigSlot {
public:
	/*********************
	** Typedefs
	**********************/
	typedef typename Topic<Data>::Subscribers Subscribers; /**< @brief A list of subscribers (slots) to a given topic. **/
	typedef typename std::map<std::string, const Subscribers*> PublicationMap; /**< @brief Stores publishing topics and their followers. **/

	/*********************
	** C&D
	**********************/
	/**
	 * Used only by signals where the function callback automatically
	 * defaults to the emit() function.
	 */
	SigSlot() : processing_count(0), number_of_handles(1) {
		function = new PartiallyBoundUnaryMemberFunction< SigSlot<Data> ,Data,void >( &SigSlot<Data>::emit, *this);
	}
	/**
	 * Used by slots loading global or static functions.
	 *
	 * @param f : the global/static function.
	 */
	SigSlot(void (*f)(Data)) : processing_count(0), number_of_handles(1) {
		function = new UnaryFreeFunction<Data>(f);
	}
	/**
	 * Used by slots loading a member function.
	 *
	 * @param f : the member function to load.
	 * @param c : the instance for the member function's class.
	 * @tparam C : the member function's class type.
	 */
	template<typename C>
	SigSlot(void (C::*f)(Data), C &c) : processing_count(0), number_of_handles(1) {
		function = new PartiallyBoundUnaryMemberFunction<C,Data,void>(f,c);
	}

	/**
	 * @brief Disconnects the sigslot completely.
	 *
	 * This is a rather elaborate process, it must first disconnect from any signals
	 * to ensure no new callbacks are instantiated, then check to see that there
	 * are no callbacks actually running, block if so, then finalise any deletions.
	 */
	~SigSlot() {
		disconnect(); // stop any new processing from connected signals
		mutex.lock(); // acts like a barrier - holds up here if function still processing stuff.
		delete function;
	}

	const unsigned int& handles() const { return number_of_handles; } /**< @brief Number of copies of this object. **/
	void incrHandles() { ++number_of_handles; } /**< @brief Increment the counter for the number of copies of this object. **/
	void decrHandles() { --number_of_handles; } /**< @brief Decrement the counter for the number of copies of this object. **/

	/**
	 * @brief Emit a signal along with the specified data.
	 *
	 * This is used by signals when emitting to slots.
	 */
	void emit(Data data) {
		typename PublicationMap::const_iterator topic_iter;
		typename Subscribers::const_iterator slots_iter;
		for ( topic_iter = publications.begin(); topic_iter != publications.end(); ++topic_iter ) {
			const Subscribers* subscribers = topic_iter->second;
			for ( slots_iter = subscribers->begin(); slots_iter != subscribers->end(); ++slots_iter ) {
				SigSlot<Data> *sigslot = *slots_iter;
				sigslot->process(data);
			}
		}
	}
	/**
	 * @brief Process the callback function loaded into this sigslot.
	 *
	 * This is used by slots with their loaded functions or relaying
	 * signals with their emit() methods.
	 */
	void process(Data data) {
		mutex.trylock(); // Only lock if its not already locked.
		++processing_count;
		(*function)(data);
		if ( --processing_count == 0 ) {
			mutex.unlock();
		}
	}
	/**
	 * @brief Connect a signal to the specified topic.
	 *
	 * If the topic doesn't exist, it will be created and the signal
	 * connected. If it does exist, it will connect with a set of
	 * slots also currently linked to the topic.
	 */
	void connectSignal(const std::string& topic) {
		// Logic:
		//   - if already publishing to this topic
		//     - don't do anything
		//   - else
		//     - if topic doesn't exist
		//       - Manager will automatically create a new topic
		//     - Manager returns the subscribers handle
		//     - Topic name and subscribers handle are stored locally here in publications
		publications.insert( std::pair<std::string, const Subscribers*>(topic, SigSlotsManager<Data>::connectSignal(topic,this)) );
	}
	/**
	 * @brief Connect a slot to the specified topic.
	 *
	 * If the topic doesn't exist, it will be created and the slot
	 * connected. If it does exist, it will pass on its link
	 * details to any signals also connected to the topic.
	 */
	void connectSlot(const std::string& topic) {
		std::pair< std::set<std::string>::iterator,bool > ret;
	//	std::cout << "Topic: " << topic << std::endl;
		ret = subscriptions.insert(topic); // Doesn't matter if it already exists.
		if ( ret.second ) {
			SigSlotsManager<Data>::connectSlot(topic,this);
		} // else { already subscribed to this topic }
	}
	/**
	 * @brief Disconnect the sigslot from the specified topic.
	 */
	void disconnect(const std::string &topic) {
		std::set<std::string>::const_iterator listen_iter = subscriptions.find(topic);
		publications.erase(topic); // Doesn't matter if it finds it or not.
		SigSlotsManager<Void>::disconnect(topic,this);
	}
	/**
	 * @brief Disconnect the sigslot from all topics.
	 *
	 * This completely disconnects the sigslot.
	 */
	void disconnect() {
		std::set<std::string>::iterator iter;
		for ( iter = subscriptions.begin(); iter != subscriptions.end(); ++iter ) {
			SigSlotsManager<Data>::disconnect(*iter, this);
		}
		subscriptions.clear();
		typename std::map<std::string,const Subscribers*>::iterator emit_iter;
		for ( emit_iter = publications.begin(); emit_iter != publications.end(); ++emit_iter ) {
			SigSlotsManager<Data>::disconnect(emit_iter->first, this);
		}
		publications.clear();
	}

	const std::set<std::string>& subscribedTopics() { return subscriptions; }

private:
	Mutex mutex;
	unsigned int processing_count; // number of running process()'
	unsigned int number_of_handles; // number of handles to this sigslot (allows copying)
	std::set<std::string> subscriptions; // topics this sigslot is listening to
	PublicationMap publications; // topics this sigslot is posting to, as well as the subscribers on the other end

	UnaryFunction<Data,void> *function;
};

/*****************************************************************************
** Interface [Void]
*****************************************************************************/

/**
 * @brief Not for direct use, provides the power behind both void signals and slots.
 *
 * This is the workhorse for both signals and slots, providing the implementation
 * for all the functions necessary by both types of frontends. This is the
 * specialisation of the general case for void callbacks.
 */
template<>
class SigSlot<Void> {
public:
	// typedef std::set<SigSlot<Void>*> Subscribers
	typedef Topic<Void>::Subscribers Subscribers; /**< @brief A list of subscribers (slots) to a given topic. **/
	typedef std::map<std::string, const Subscribers*> PublicationMap; /**< @brief Stores publishing topics and their followers. **/

	/**
	 * Used only by signals where the function callback automatically
	 * defaults to the emit() function.
	 */
	SigSlot() : processing_count(0), number_of_handles(1) {
		function = new BoundNullaryMemberFunction<SigSlot,void>(&SigSlot::emit,*this);
	}
	/**
	 * Used by slots loading global or static functions.
	 *
	 * @param f : the global/static function.
	 */
	SigSlot(VoidFunction f) : processing_count(0), number_of_handles(1) {
		function = new NullaryFreeFunction<void>(f);
	}
	/**
	 * Used by slots loading a member function.
	 *
	 * @param f : the member function to load.
	 * @param c : the instance for the member function's class.
	 * @tparam C : the member function's class type.
	 */
	template<typename C>
	SigSlot(void (C::*f)(void), C &c) : processing_count(0), number_of_handles(1) {
		function = new BoundNullaryMemberFunction<C,void>(f,c);
	}

	/**
	 * @brief Disconnects the sigslot completely.
	 *
	 * This is a rather elaborate process, it must first disconnect from any signals
	 * to ensure no new callbacks are instantiated, then check to see that there
	 * are no callbacks actually running, block if so, then finalise any deletions.
	 */
	~SigSlot() {
		disconnect(); // stop any new processing from connected signals
		mutex.lock(); // acts like a barrier - holds up here if function still processing stuff.
		delete function;
	}

	const unsigned int& handles() const { return number_of_handles; } /**< @brief Number of copies of this object. **/
	void incrHandles() { ++number_of_handles; } /**< @brief Increment the counter for the number of copies of this object. **/
	void decrHandles() { --number_of_handles; } /**< @brief Decrement the counter for the number of copies of this object. **/

	/**
	 * @brief Emit a signal.
	 *
	 * This is used by signals when emitting to slots.
	 */
	void emit() {
		PublicationMap::const_iterator topic_iter;
		Subscribers::const_iterator slots_iter;
		for ( topic_iter = publications.begin(); topic_iter != publications.end(); ++topic_iter ) {
			const Subscribers* subscribers = topic_iter->second;
			for ( slots_iter = subscribers->begin(); slots_iter != subscribers->end(); ++slots_iter ) {
				SigSlot *sigslot = *slots_iter;
				sigslot->process();
			}
		}
	}
	/**
	 * @brief Process the callback function loaded into this sigslot.
	 *
	 * This is used by slots with their loaded functions or relaying
	 * signals with their emit() methods.
	 */
	void process(Void void_arg = Void()) {
		mutex.trylock(); // Only lock if its not already locked.
		++processing_count;
		(*function)();
		if ( --processing_count == 0 ) {
			mutex.unlock();
		}
	}
	/**
	 * @brief Connect a signal to the specified topic.
	 *
	 * If the topic doesn't exist, it will be created and the signal
	 * connected. If it does exist, it will connect with a set of
	 * slots also currently linked to the topic.
	 */
	void connectSignal(const std::string& topic) {
		// Logic:
		//   - if already publishing to this topic
		//     - don't do anything
		//   - else
		//     - if topic doesn't exist
		//       - Manager will automatically create a new topic
		//     - Manager returns the subscribers handle
		//     - Topic name and subscribers handle are stored locally here in publications
		publications.insert( std::pair<std::string, const Subscribers*>(topic, SigSlotsManager<Void>::connectSignal(topic,this)) );
	}
	/**
	 * @brief Connect a slot to the specified topic.
	 */
	void connectSlot(const std::string& topic){
		std::pair< std::set<std::string>::iterator,bool > ret;
		ret = subscriptions.insert(topic); // Doesn't matter if it already exists.
		if ( ret.second ) {
			SigSlotsManager<Void>::connectSlot(topic,this);
		} // else { already subscribed to this topic }
	}
	/**
	 * @brief Disconnect the sigslot from the specified topic.
	 *
	 * If the topic doesn't exist, it will be created and the slot
	 * connected. If it does exist, it will pass on its link
	 * details to any signals also connected to the topic.
	 */
	void disconnect(const std::string &topic) {
		std::set<std::string>::const_iterator listen_iter = subscriptions.find(topic);
		publications.erase(topic); // Doesn't matter if it finds it or not.
		SigSlotsManager<Void>::disconnect(topic,this);
	}
	/**
	 * @brief Disconnect the sigslot from all topics.
	 *
	 * This completely disconnects the sigslot.
	 */
	void disconnect() {
		std::set<std::string>::iterator iter;
		for ( iter = subscriptions.begin(); iter != subscriptions.end(); ++iter ) {
			SigSlotsManager<Void>::disconnect(*iter, this);
		}
		subscriptions.clear();
		std::map<std::string,const Subscribers*>::iterator emit_iter;
		for ( emit_iter = publications.begin(); emit_iter != publications.end(); ++emit_iter ) {
			SigSlotsManager<Void>::disconnect(emit_iter->first, this);
		}
		publications.clear();
	}

private:
	Mutex mutex;
	unsigned int processing_count; // number of running process()'
	unsigned int number_of_handles; // number of handles to this sigslot (allows copying)
	std::set<std::string> subscriptions; // topics this sigslot is listening to
	PublicationMap publications; // topics this sigslot is posting to, as well as the subscribers on the other end

	NullaryFunction<void> *function;
};

} // namespace ecl

#endif /* ECL_SIGSLOTS_SIGSLOT_HPP_ */
