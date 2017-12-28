/**
 * @file /ecl_sigslots/include/ecl/sigslots/manager.hpp
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

#ifndef ECL_SIGSLOTS_MANAGER_HPP_
#define ECL_SIGSLOTS_MANAGER_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include <map>
#include <string>
#include <ecl/exceptions/standard_exception.hpp>
#include <ecl/config/macros.hpp>
#include <ecl/utilities/void.hpp>
#include "topic.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Interface
*****************************************************************************/
/**
 * @brief The sigslots connection manager.
 *
 * This handles all the connections via the unique string identifiers. It does
 * this invisibly, so the programmer need not actually have to use this
 * class. However, it may be useful for debugging to actually check the number
 * of connections with the printStatistics() method.
 *
 * @tparam Data : the type of sigslots this manager looks after.
 */
template<typename Data = Void>
class SigSlotsManager {
public:

	friend class SigSlot<Data>;

	/**
	 * @brief Print some statistics on the current status of the manager.
	 *
	 * Only here for debugging purposes only.
	 */
	static void printStatistics() {
		std::cout << "Topics" << std::endl;
		typename std::map< std::string, Topic<Data> >::iterator iter;
		for ( iter = topics().begin(); iter != topics().end(); ++iter ) {
			std::cout << iter->second;
		}
	}

private:
	/**
	 * @brief A list of subscribers (slots) to a given topic.
	 */
	typedef typename Topic<Data>::Subscribers Subscribers;

	/**
	 * Connects the signal to the topic if it already exists, or creates
	 * the topic if it doesn't. Returns the subscriber list which is used by
	 * sigslot to bypass the manager when emitting.
	 *
	 * @param topic : topic to publish to.
	 * @param sigslot : sigslot that will be publishing.
	 * @return const Subscribers& : a reference to the subscriber list.
	 */
	static const Subscribers* connectSignal(const std::string& topic, SigSlot<Data>* sigslot) {
		// Try and insert a new topic in case it doesn't already exist
		// In any case, we always get the iterator back (to new or existing)
		//
		// Maybe improve the efficiency of this by specifying position
		// refer to http://www.cplusplus.com/reference/stl/map/insert/
		std::pair< typename std::map< std::string, Topic<Data> >::iterator,bool> ret = topics().insert( std::pair< std::string, Topic<Data> >(topic, Topic<Data>(topic)) );
		Topic<Data>& current_topic = (ret.first)->second;
		current_topic.addPublisher(sigslot);
		return current_topic.subscribers();
	}

	/**
	 * Connects the slot to the topic if it already exists, or creates
	 * the topic if it doesn't.
	 *
	 * @param topic : topic to subscribe (listen) to.
	 * @param sigslot : sigslot that will be subscribing (listening).
	 */
	static void connectSlot(const std::string& topic, SigSlot<Data>* sigslot) {
		// Try and insert a new topic in case it doesn't already exist
		// In any case, we always get the iterator back (to new or existing)
		//
		// Maybe improve the efficiency of this by specifying position
		// refer to http://www.cplusplus.com/reference/stl/map/insert/
		std::pair< typename std::map< std::string, Topic<Data> >::iterator,bool> ret = topics().insert(std::pair< std::string, Topic<Data> >(topic, Topic<Data>(topic)) );
		Topic<Data>& current_topic = (ret.first)->second;
		current_topic.addSubscriber(sigslot);
	}

	/**
	 * @brief Disconnect the sigslot from the specified topic.
	 *
	 * This disconnection works for both signals and slots.
	 *
	 * @param topic : topic that the sigslot must be disconnected from.
	 * @param sigslot : the sigslots that is to be disconnected.
	 */
	static void disconnect(const std::string& topic, SigSlot<Data>* sigslot) {
		typename std::map<std::string, Topic<Data> >::iterator iter = topics().find(topic);
		if ( iter != topics().end() ) {
			iter->second.disconnect(sigslot);
		}
		if ( iter->second.empty() ) {
			topics().erase(iter);
		}
	}

	/**
	 * @brief Check to see if the specified topic exists (and is being used).
	 *
	 * @param topic : topic to check.
	 * @return bool : success/failure of the request.
	 */
	static bool isTopic(const std::string& topic) {
		return !( topics().find(topic) == topics().end() );
	}

	/**
	 * @brief Hack to create a static variable internally without.
	 *
	 * Simple trick to avoid the explicit instantiation in a library.
	 *
	 * @return map : a handle to the string id/topic database.
	 */
	static std::map< std::string, Topic<Data> >& topics() {
		static std::map< std::string, Topic<Data> > topic_list;
		return topic_list;
	}

	/**
	 * @brief Provides a list of subscribers (listeners) associated with a topic.
	 *
	 * This provides a handle to the list of subscribers to a topic. Used by
	 * the signals to keep track of who's following them and who to run when
	 * emitting.
	 *
	 * @param topic : the topic to check for.
	 * @return Subscribers : a set of pointers to subscribers of a topic.
	 */
	static const Subscribers& subscribers(const std::string& topic) {
		typename std::map< std::string, Topic<Data> >::const_iterator iter = topics().find(topic);
		/*
		 * Note that this is called only by SigSlotsManager::connectSignal which
		 * makes sure the topic name exists, so we don't need to do any error
		 * handling here.
		 */
		// ecl_assert_throw( iter != topics().end(), StandardException(LOC,InvalidInputError,std::string("No sigslots topic with name:")+topic) );
		return *iter->second.subscribers();
	}

};

} // namespace ecl


#endif /* ECL_SIGSLOTS_MANAGER_HPP_ */
