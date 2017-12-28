/**
 * @file /ecl_sigslots/include/ecl/sigslots/topic.hpp
 *
 * @brief Simple structure holding publishers and subscribers to a topic.
 *
 * @date 13/05/2010
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ECL_SIGSLOTS_TOPIC_HPP_
#define ECL_SIGSLOTS_TOPIC_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <string>
#include <set>
#include <ecl/config/macros.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Forward Declarations
*****************************************************************************/

template <typename Data>
class SigSlot;

/*****************************************************************************
** Interface
*****************************************************************************/
/**
 * @brief Stores publisher and subscriber lists to a uniquely string identified topic.
 *
 * This is not for direct use, its the storage container for the sigslots manager.
 */
template <typename Data>
class ECL_LOCAL Topic {
public:
	/*********************
	** Typedefs
	**********************/
	typedef std::set<SigSlot<Data>*> Subscribers;  /**< @brief A list of subscribers (slots) to a given topic. **/

	/**
	 * @brief Uniquely construct with the specified name.
	 * @param name : name of the topic.
	 */
	Topic(const std::string& name) : topic_name(name) {}

	/**
	 * @brief List of subscribers (listeners) to a topic.
	 * @return Subscribers : handle to the list.
	 */
	const Subscribers* subscribers() const { return &topic_subscribers; }

	/**
	 * @brief Add a subscriber.
	 * @param sigslot : the sigslot to add.
	 */
	void addSubscriber(SigSlot<Data>* sigslot) {
		topic_subscribers.insert(sigslot);
	}
	/**
	 * @brief Add a publisher.
	 * @param sigslot : the sigslot to add.
	 */
	void addPublisher(SigSlot<Data>* sigslot) {
		topic_publishers.insert(sigslot);
	}
	/**
	 * @brief Disconnect a sigslot.
	 *
	 * This parses both publishers and subscribers looking for the sigslot to connect.
	 * Could be a bit heavy doing it this way, but its convenient.
	 *
	 * @param sigslot : the sigslot to disconnect.
	 */
	void disconnect(SigSlot<Data>* sigslot) {
		typename std::set<SigSlot<Data>*>::const_iterator iter = topic_publishers.find(sigslot);
		if ( iter != topic_publishers.end() ) {
			topic_publishers.erase(iter);
		}
		iter = topic_subscribers.find(sigslot);
		if ( iter != topic_subscribers.end() ) {
			topic_subscribers.erase(iter);
		}
	}
	/**
	 * @brief Checks to see if there is no publishers/subscribers.
	 *
	 * This is used to delete the topic (by the manager) if it is empty.
	 * @return bool : empty (true) or not (false).
	 */
	bool empty() const {
		return ( ( topic_publishers.size() == 0 ) && ( topic_subscribers.size() == 0 ) );
	}

    /*********************
    ** Streaming
    **********************/
    /**
     * Insertion operator for sending the topic to an output stream.
     * @param ostream : the output stream.
     * @param topic : the topic to be inserted.
     * @return OutputStream : continue streaming with the updated output stream.
     */
    template <typename OutputStream, typename TopicData>
    friend OutputStream& operator<<(OutputStream &ostream , const Topic<TopicData>& topic);

private:
    std::string topic_name;
    std::set<SigSlot<Data>*> topic_publishers;
    std::set<SigSlot<Data>*> topic_subscribers;
};


/*****************************************************************************
** Implementation [Streaming]
*****************************************************************************/

template <typename OutputStream, typename TopicData>
OutputStream& operator<<(OutputStream &ostream , const Topic<TopicData> &topic) {

	ostream << "  Name: " << topic.topic_name << "\n";
	ostream << "    # Subscribers: " << topic.topic_subscribers.size() << "\n";
	ostream << "    # Publishers : " << topic.topic_publishers.size() << "\n";
    ostream.flush();

    return ostream;
}

} // namespace ecl

#endif /* ECL_SIGSLOTS_TOPIC_HPP_ */
