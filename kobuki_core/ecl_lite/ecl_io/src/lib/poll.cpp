/**
 * @file /ecl_io/src/lib/poll.cpp
 *
 * @brief Implementation for a cross platform polling function.
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../../include/ecl/io/poll.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace ecl {

/*****************************************************************************
** Implementation
*****************************************************************************/

int poll_sockets(socket_pollfd *fds, nfds_t nfds, int timeout) {
#if defined(ECL_IS_WIN32)
	fd_set readfds, writefds, exceptfds;
	struct timeval tv, *ptv;
	socket_descriptor max_fd;
	int rc;
	nfds_t i;

	if (fds == NULL) {
		errno = EFAULT;
		return -1;
	}

	FD_ZERO (&readfds);
	FD_ZERO (&writefds);
	FD_ZERO (&exceptfds);

	/*********************
	** Compute fd sets
	**********************/
	// also find the largest descriptor.
	for (rc = -1, max_fd = 0, i = 0; i < nfds; i++) {
		if (fds[i].fd == INVALID_SOCKET) {
			continue;
		}
		if (fds[i].events & (POLLIN | POLLRDNORM)) {
			FD_SET (fds[i].fd, &readfds);
		}
		if (fds[i].events & (POLLOUT | POLLWRNORM | POLLWRBAND)) {
			FD_SET (fds[i].fd, &writefds);
		}
		if (fds[i].events & (POLLPRI | POLLRDBAND)) {
			FD_SET (fds[i].fd, &exceptfds);
		}
		if (fds[i].fd > max_fd &&
			  (fds[i].events & (POLLIN | POLLOUT | POLLPRI |
								POLLRDNORM | POLLRDBAND |
								POLLWRNORM | POLLWRBAND))) {
			max_fd = fds[i].fd;
			rc = 0;
		}
	}

	if (rc == -1) {
		errno = EINVAL;
		return -1;
	}
	/*********************
	** Setting the timeout
	**********************/
	if (timeout < 0) {
		ptv = NULL;
	} else {
		ptv = &tv;
		if (timeout == 0) {
			tv.tv_sec = 0;
			tv.tv_usec = 0;
		} else {
			tv.tv_sec = timeout / 1000;
			tv.tv_usec = (timeout % 1000) * 1000;
		}
	}

	rc = select (max_fd + 1, &readfds, &writefds, &exceptfds, ptv);
	if (rc < 0) {
		return -1;
	} else if ( rc == 0 ) {
		return 0;
	}

	for (rc = 0, i = 0; i < nfds; i++) {
		if (fds[i].fd != INVALID_SOCKET) {
			fds[i].revents = 0;

			if (FD_ISSET(fds[i].fd, &readfds)) {
				int save_errno = errno;
				char data[64] = {0};
				int ret;

				/* support for POLLHUP */
				// just check if there's incoming data, without removing it from the queue.
				ret = recv(fds[i].fd, data, 64, MSG_PEEK);
				#ifdef WIN32
				if ((ret == -1) &&
						(errno == WSAESHUTDOWN || errno == WSAECONNRESET ||
						(errno == WSAECONNABORTED) || errno == WSAENETRESET))
				#else
				if ((ret == -1) &&
						(errno == ESHUTDOWN || errno == ECONNRESET ||
						(errno == ECONNABORTED) || errno == ENETRESET))
				#endif
				{
					fds[i].revents |= POLLHUP;
				} else {
					fds[i].revents |= fds[i].events & (POLLIN | POLLRDNORM);
				}
				errno = save_errno;
			}
			if (FD_ISSET(fds[i].fd, &writefds)) {
				fds[i].revents |= fds[i].events & (POLLOUT | POLLWRNORM | POLLWRBAND);
			}

			if (FD_ISSET(fds[i].fd, &exceptfds)) {
				fds[i].revents |= fds[i].events & (POLLPRI | POLLRDBAND);
			}

			if (fds[i].revents & ~POLLHUP) {
				rc++;
			}
		} else {
				fds[i].revents = POLLNVAL;
		}
	}
	return rc;
#else
	// should really put in a cmake check to check that the poll function is present
	// --> do it when necessary.

	// use an existing poll implementation
	int result = poll(fds, nfds, timeout);
	if ( result < 0 ) {
		// EINTR means that we got interrupted by a signal, and is not an error
		if(errno == EINTR) {
			result = 0;
		}
	}
	return result;
#endif // poll_sockets functions
}

} // namespace ecl
